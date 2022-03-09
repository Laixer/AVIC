// SPDX-License-Identifier: GPL-2.0
/*
 * AVIC CAN driver.
 *
 * Copyright (C) 2021-2022 Yorick de Wid (yorick@laixer.com)
 * Copyright (C) 2021-2022 Laixer Equipment B.V.
 */

// TODO:
// - More read buffers
// - Locking
// - Extended frame
// - Set bit timing

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/version.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>

#include "avic.h"

/* Driver identifier */
#define DRV_NAME "avic_can"

/* USB interrupt requests */
#define USB_REQ_DUMP_INFO 2     /* Request info dump to console output. */
#define USB_REQ_SYNC_CLOCK 7    /* Clock synchronization request. */
#define USB_REQ_SYSTEM_RESET 9  /* System reset request. */
#define USB_REQ_HALT_MOTION 128 /* Halt all motor functions. */

/* AVIC frame default port. Can change in the future */
#define AVIC_PORT_DEFAULT 0

/* AVIC frame types */
#define AVIC_FRAME_TYPE_CAN 0x8       /* CAN bus frame */
#define AVIC_FRAME_TYPE_CAN_FD 0x9    /* CAN FD bus frame */
#define AVIC_FRAME_TYPE_ETHERNET 0x11 /* Ethernet frame */

#define TX_MAX_CONTENT_SLOTS 32
#define RX_MAX_CONTENT_SLOTS 32

#define MIN_BULK_PACKET_SIZE 64

struct avic_usb_tx_urb_context
{
    struct avic_bridge *dev;
    unsigned int is_free;
    unsigned int index;
};

struct avic_bridge
{
    /* The can-dev module expects this member. */
    struct can_priv can;

    struct usb_device *udev;
    struct net_device *netdev;

    struct usb_anchor tx_submitted;
    struct usb_anchor rx_submitted;

    struct avic_usb_endpoint_info write_ep;
    struct avic_usb_endpoint_info read_ep;

    struct avic_usb_tx_urb_context tx_context[TX_MAX_CONTENT_SLOTS];
};

/* AVIC frame which carries the payload. */
struct avic_frame
{
    __u8 type;     /* Frame type */
    __u32 id;      /* Source address */
    __u8 port;     /* AVIC port */
    __u8 len;      /* Payload length */
    __u8 data[48]; /* Payload */
} __attribute__((packed));

static void avic_usb_write_bulk_callback(struct urb *urb)
{
    struct avic_usb_tx_urb_context *context = urb->context;
    struct avic_bridge *dev = context->dev;
    struct net_device *netdev = dev->netdev;

    /* Free the buffer as soon as possible */
    usb_free_coherent(urb->dev, urb->transfer_buffer_length,
                      urb->transfer_buffer, urb->transfer_dma);

    if (!netif_device_present(netdev))
    {
        return;
    }

    switch (urb->status)
    {
    case 0: /* success */
        break;

    case -ECONNRESET: /* unlink */
    case -ENOENT:
    case -EPIPE:
    case -EPROTO:
    case -ESHUTDOWN:
        return;

    default:
        pr_warn("tx bulk aborted: %d\n", urb->status);
        return;
    }

    /* At this point the transmission was a success so update the stats */
    netdev->stats.tx_packets++;
    netdev->stats.tx_bytes += urb->actual_length;

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 12, 0)
    can_get_echo_skb(netdev, context->index);
#else
    can_get_echo_skb(netdev, context->index, NULL);
#endif

    context->is_free = 1;

    netif_wake_queue(netdev);
}

static netdev_tx_t avic_can_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    struct avic_bridge *dev = netdev_priv(netdev);
    struct canfd_frame *frame = (struct canfd_frame *)skb->data;
    struct urb *urb = NULL;
    struct avic_frame *avic_frame = NULL;
    struct avic_usb_tx_urb_context *context = NULL;
    u8 *buf = NULL;
    int i = 0, retval = -ENOMEM;

    /* Drop non-CAN frames */
    if (can_dropped_invalid_skb(netdev, skb))
    {
        netdev->stats.tx_dropped++;
        return NETDEV_TX_OK;
    }

    urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!urb)
    {
        pr_err("usb_alloc_urb failed");
        netdev->stats.tx_dropped++;
        return retval;
    }

    buf = usb_alloc_coherent(dev->udev, dev->write_ep.max_packet_size, GFP_ATOMIC, &urb->transfer_dma);
    if (!buf)
    {
        pr_err("usb_alloc_coherent failed");

        goto cleanup_urb;
    }

    /* Convert the canfd_frame into an avic_frame */
    avic_frame = (struct avic_frame *)buf;
    avic_frame->type = AVIC_FRAME_TYPE_CAN_FD;
    avic_frame->id = frame->can_id;
    avic_frame->port = AVIC_PORT_DEFAULT;
    avic_frame->len = frame->len;
    memcpy(avic_frame->data, frame->data, frame->len);

    for (i = 0; i < TX_MAX_CONTENT_SLOTS; ++i)
    {
        if (dev->tx_context[i].is_free)
        {
            context = &dev->tx_context[i];
            context->dev = dev;
            context->index = i;
            context->is_free = 0;
            break;
        }
    }

    if (!context)
    {
        pr_warn("no available context slots\n");

        usb_free_coherent(dev->udev, dev->write_ep.max_packet_size, buf, urb->transfer_dma);
        usb_free_urb(urb);

        /*
         * All slots are in-flight which is unusual even on high throughput connections. If we
         * signal a busy status to netdev it will keep retrying to send the same packet after awhile.
         *
         * Slow down the network interface by stopping the TX queue. Any consecutive callback will wake
         * the TX queue. This is an effective way to implement flow control.
         */
        netif_stop_queue(netdev);

        netdev->stats.tx_dropped++;
        return NETDEV_TX_BUSY;
    }

    usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->write_ep.address),
                      buf, dev->write_ep.max_packet_size,
                      avic_usb_write_bulk_callback, context);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->tx_submitted);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 12, 0)
    can_put_echo_skb(skb, netdev, context->index);
#else
    can_put_echo_skb(skb, netdev, context->index, 0);
#endif

    /* send the data out the bulk port */
    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);

        goto cleanup_buffer;
    }

    usb_free_urb(urb);

    return NETDEV_TX_OK;

cleanup_buffer:
    usb_free_coherent(dev->udev, dev->write_ep.max_packet_size, buf, urb->transfer_dma);

cleanup_urb:
    usb_free_urb(urb);

    netdev->stats.tx_dropped++;

    return retval;
}

static int avic_can_open(struct net_device *netdev)
{
    int err = open_candev(netdev);
    if (err)
    {
        return err;
    }

    /* Accept packets on the network queue */
    netif_start_queue(netdev);

    return 0;
}

static int avic_can_close(struct net_device *netdev)
{
    /* We'll no longer accept new packets */
    netif_stop_queue(netdev);

    close_candev(netdev);

    return 0;
}

static const struct net_device_ops avic_can_netdev_ops = {
    .ndo_open = avic_can_open,
    .ndo_stop = avic_can_close,
    .ndo_start_xmit = avic_can_start_xmit,
    .ndo_change_mtu = can_change_mtu,
};

static const struct can_bittiming_const avic_can_bittiming_const = {
    .name = "avic_can",
    .tseg1_min = 1,
    .tseg1_max = 16,
    .tseg2_min = 1,
    .tseg2_max = 8,
    .sjw_max = 4,
    .brp_min = 1,
    .brp_max = 64,
    .brp_inc = 1,
};

static int avic_can_set_mode(struct net_device *netdev, enum can_mode mode)
{
    switch (mode)
    {
    case CAN_MODE_START:
        break;

    default:
        return -EOPNOTSUPP;
    }

    return 0;
}

static int avic_can_set_bittiming(struct net_device *netdev)
{
    /* TODO: Send the bittime change to the AVIC */
    return 0;
}

static void avic_usb_read_bulk_callback(struct urb *urb)
{
    struct avic_bridge *dev = urb->context;
    struct net_device *netdev = dev->netdev;
    struct avic_frame *avic_frame = NULL;
    struct can_frame *frame = NULL;
    struct sk_buff *skb = NULL;
    int retval = 0;

    if (!netif_device_present(netdev))
    {
        return;
    }

    switch (urb->status)
    {
    case 0: /* success */
        break;

    case -ECONNRESET: /* unlink */
    case -ENOENT:
    case -EPIPE:
    case -EPROTO:
    case -ESHUTDOWN:
        return;

    default:
        pr_warn("rx bulk aborted: %d\n", urb->status);
        return;
    }

    avic_frame = (struct avic_frame *)urb->transfer_buffer;

    skb = alloc_can_skb(netdev, &frame);
    if (!skb)
    {
        pr_err("alloc_can_skb failed\n");
        return;
    }

#if LINUX_VERSION_CODE <= KERNEL_VERSION(5, 12, 0)
    frame->can_dlc = avic_frame->len;
#else
    frame->len = avic_frame->len;
#endif
    memcpy(frame->data, avic_frame->data, avic_frame->len);

    /* At this point the receive was a success so update the stats */
    netdev->stats.rx_packets++;
    netdev->stats.rx_bytes += urb->actual_length;

    netif_rx(skb);

    usb_fill_bulk_urb(urb, dev->udev,
                      usb_rcvbulkpipe(dev->udev, dev->read_ep.address),
                      urb->transfer_buffer, dev->read_ep.max_packet_size,
                      avic_usb_read_bulk_callback, dev);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->rx_submitted);

    retval = usb_submit_urb(urb, GFP_KERNEL);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);
    }
}

static int avic_can_configure_peripheral(struct avic_bridge *dev)
{
    struct urb *urb = NULL;
    u8 *buf = NULL;
    int retval = -ENOMEM;

    // TODO: Move to setup. We need to release this in the discon.
    urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!urb)
    {
        return retval;
    }

    // TODO: Move to setup. We need to release this in the discon.
    buf = usb_alloc_coherent(dev->udev, dev->read_ep.max_packet_size, GFP_ATOMIC, &urb->transfer_dma);
    if (!buf)
    {
        pr_err("no memory left for USB buffer\n");
        goto cleanup_urb;
    }

    usb_fill_bulk_urb(urb, dev->udev,
                      usb_rcvbulkpipe(dev->udev, dev->read_ep.address),
                      buf, dev->read_ep.max_packet_size,
                      avic_usb_read_bulk_callback, dev);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->rx_submitted);

    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);

        goto cleanup_buffer;
    }

    usb_free_urb(urb);

    return 0;

cleanup_buffer:
    usb_free_coherent(dev->udev, dev->read_ep.max_packet_size, buf, urb->transfer_dma);

cleanup_urb:
    usb_free_urb(urb);

    return retval;
}

static int avic_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
    struct net_device *netdev = NULL;
    struct avic_bridge *dev = NULL;
    struct usb_endpoint_descriptor *read_in = NULL, *write_out = NULL;
    int i = 0, retval = -ENOMEM;

    pr_info("found AVIC CAN interface\n");

    netdev = alloc_candev(sizeof(struct avic_bridge), 32);
    if (!netdev)
    {
        pr_err("could not allocate candev");
        return retval;
    }

    dev = netdev_priv(netdev);

    dev->udev = interface_to_usbdev(intf);
    dev->netdev = netdev;

    dev->can.state = CAN_STATE_STOPPED;
    dev->can.clock.freq = 8000000; // TODO: define
    dev->can.bittiming_const = &avic_can_bittiming_const;
    dev->can.do_set_bittiming = avic_can_set_bittiming;
    dev->can.do_set_mode = avic_can_set_mode;
    dev->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES;

    init_usb_anchor(&dev->rx_submitted);
    init_usb_anchor(&dev->tx_submitted);

    for (i = 0; i < TX_MAX_CONTENT_SLOTS; ++i)
    {
        dev->tx_context[i].is_free = 1;
    }

    netdev->netdev_ops = &avic_can_netdev_ops;

    netdev->flags |= IFF_ECHO; /* Local echo support */

    /*
     * Look for the bulk endpoints in the current interface
     * descriptor. These endpoints *must* exist in this interface.
     */
    retval = usb_find_common_endpoints(intf->cur_altsetting,
                                       &read_in,
                                       &write_out,
                                       NULL, NULL);
    if (retval)
    {
        pr_err("did not find bulk endpoints\n");
        goto cleanup_candev;
    }

    dev->write_ep.address = usb_endpoint_num(write_out);
    dev->write_ep.max_packet_size = usb_endpoint_maxp(write_out);

    if (dev->write_ep.max_packet_size < MIN_BULK_PACKET_SIZE)
    {
        pr_err("write bulk max size too small");

        retval = -ENODEV;
        goto cleanup_candev;
    }

    dev->read_ep.address = usb_endpoint_num(read_in);
    dev->read_ep.max_packet_size = usb_endpoint_maxp(read_in);

    if (dev->read_ep.max_packet_size < MIN_BULK_PACKET_SIZE)
    {
        pr_err("read bulk max size too small");

        retval = -ENODEV;
        goto cleanup_candev;
    }

    /* Entangle the USB interface with the AVIC bridge device */
    usb_set_intfdata(intf, dev);

    SET_NETDEV_DEV(netdev, &intf->dev);

    retval = register_candev(netdev);
    if (retval)
    {
        pr_err("could not register CAN device: %d\n", retval);

        goto cleanup_candev;
    }

    return avic_can_configure_peripheral(dev);

cleanup_candev:
    free_candev(netdev);

    return retval;
}

/*
 * Called by the usb core when the device is removed from the system.
 *
 * This method will release all unmanaged resources back to the kernel.
 */
static void avic_usb_disconnect(struct usb_interface *intf)
{
    struct avic_bridge *dev = usb_get_intfdata(intf);

    usb_set_intfdata(intf, NULL);

    if (dev)
    {
        unregister_candev(dev->netdev);

        free_candev(dev->netdev);
    }
}

/* Table of devices that work with the AVIC CAN driver. */
static struct usb_device_id avic_usb_table[] = {
    {USB_DEVICE_AND_INTERFACE_INFO(AVIC_BRIDGE_VENDOR_ID,
                                   AVIC_BRIDGE_PRODUCT_ID,
                                   AVIC_BRIDGE_IFACE_CLASS,
                                   AVIC_BRIDGE_IFACE_SUBCLASS_DATA,
                                   AVIC_BRIDGE_IFACE_PROTO)},
    {} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, avic_usb_table);

static struct usb_driver avic_usb_driver = {
    .name = DRV_NAME,
    .probe = avic_usb_probe,
    .disconnect = avic_usb_disconnect,
    .id_table = avic_usb_table,
};

module_usb_driver(avic_usb_driver);

MODULE_AUTHOR("Yorick de Wid <yorick@laixer.com>");
MODULE_AUTHOR("Laixer Equipment B.V.");
MODULE_DESCRIPTION("AVIC Bridge driver");
MODULE_LICENSE("GPL");
