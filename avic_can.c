// SPDX-License-Identifier: GPL-2.0
/*
 * AVIC CAN driver.
 *
 * Copyright (C) 2021-2022 Yorick de Wid (yorick@laixer.com)
 * Copyright (C) 2021-2022 Laixer Equipment B.V.
 */

// TODO:
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

/* AVIC frame default port. Can change in the future */
#define AVIC_PORT_DEFAULT 0

/* AVIC frame types */
#define AVIC_FRAME_TYPE_CAN 0x8    /* CAN bus frame */
#define AVIC_FRAME_TYPE_CAN_FD 0x9 /* CAN FD bus frame */

#define TX_MAX_CONTENT_SLOTS 32
#define RX_MAX_CONTENT_SLOTS 32

#define MIN_BULK_PACKET_SIZE 64

/* AVIC device CAN clock */
#define AVIC_USB_ABP_CLOCK 32000000

struct avic_bridge
{
    /* The can-dev module expects this member. */
    struct can_priv can;

    struct usb_device *udev;
    struct net_device *netdev;

    atomic_t tx_active;
    struct usb_anchor tx_submitted;
    struct usb_anchor rx_submitted;

    struct avic_usb_endpoint_info write_ep;
    struct avic_usb_endpoint_info read_ep;

    void *rxbuf[RX_MAX_CONTENT_SLOTS];
    dma_addr_t rxbuf_dma[RX_MAX_CONTENT_SLOTS];
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
    struct avic_bridge *dev = urb->context;
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
        netdev_warn(netdev, "tx bulk aborted: %d\n", urb->status);
        return;
    }

    /* At this point the transmission was a success so update the stats */
    netdev->stats.tx_packets++;
    netdev->stats.tx_bytes += urb->actual_length;

    can_led_event(netdev, CAN_LED_EVENT_TX);

    atomic_dec(&dev->tx_active);

    netif_wake_queue(netdev);
}

static netdev_tx_t avic_can_start_xmit(struct sk_buff *skb, struct net_device *netdev)
{
    struct avic_bridge *dev = netdev_priv(netdev);
    struct canfd_frame *frame = (struct canfd_frame *)skb->data;
    struct urb *urb = NULL;
    struct avic_frame *avic_frame = NULL;
    u8 *buf = NULL;
    int retval = -ENOMEM;

    /* Drop non-CAN frames */
    if (can_dropped_invalid_skb(netdev, skb))
    {
        netdev->stats.tx_dropped++;
        return NETDEV_TX_OK;
    }

    urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!urb)
    {
        netdev_err(netdev, "usb_alloc_urb failed");
        netdev->stats.tx_dropped++;
        return retval;
    }

    buf = usb_alloc_coherent(dev->udev, dev->write_ep.max_packet_size, GFP_ATOMIC, &urb->transfer_dma);
    if (!buf)
    {
        netdev_err(netdev, "usb_alloc_coherent failed");

        goto cleanup_urb;
    }

    /* Convert the canfd_frame into an avic_frame */
    avic_frame = (struct avic_frame *)buf;
    avic_frame->type = AVIC_FRAME_TYPE_CAN_FD;
    avic_frame->id = frame->can_id;
    avic_frame->port = AVIC_PORT_DEFAULT;
    avic_frame->len = frame->len;
    memcpy(avic_frame->data, frame->data, frame->len);

    kfree_skb(skb);

    usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->write_ep.address),
                      buf, dev->write_ep.max_packet_size,
                      avic_usb_write_bulk_callback, dev);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->tx_submitted);

    atomic_inc(&dev->tx_active);

    /* Send the data out the bulk port */
    retval = usb_submit_urb(urb, GFP_ATOMIC);
    if (unlikely(retval))
    {
        netdev_err(netdev, "usb_submit_urb failed: %d\n", retval);

        goto cleanup_buffer;
    }

    if (atomic_read(&dev->tx_active) >= TX_MAX_CONTENT_SLOTS)
    {
        netdev_warn(netdev, "periperal backpressure, slow down TX queue\n");

        /*
         * All slots are in-flight which is unusual even on high throughput connections. If we
         * signal a busy status to netdev it will keep retrying to send the same packet after awhile.
         *
         * Slow down the network interface by stopping the TX queue. Any consecutive callback will wake
         * the TX queue. This is an effective way to implement flow control.
         */
        netif_stop_queue(netdev);
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
        netdev_warn(netdev, "rx bulk aborted: %d\n", urb->status);
        return;
    }

    avic_frame = (struct avic_frame *)urb->transfer_buffer;

    skb = alloc_can_skb(netdev, &frame);
    if (!skb)
    {
        netdev_err(netdev, "alloc_can_skb failed\n");
        return;
    }

    frame->can_id = avic_frame->id;
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

    can_led_event(netdev, CAN_LED_EVENT_RX);

    usb_fill_bulk_urb(urb, dev->udev,
                      usb_rcvbulkpipe(dev->udev, dev->read_ep.address),
                      urb->transfer_buffer, dev->read_ep.max_packet_size,
                      avic_usb_read_bulk_callback, dev);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->rx_submitted);

    retval = usb_submit_urb(urb, GFP_KERNEL);
    if (unlikely(retval))
    {
        netdev_err(netdev, "usb_submit_urb failed: %d\n", retval);
    }
}

static int avic_can_netif_init(struct net_device *netdev)
{
    struct avic_bridge *dev = netdev_priv(netdev);
    struct urb *urb = NULL;
    u8 *buf = NULL;
    int i = 0;
    int retval = -ENOMEM;

    for (i = 0; i < RX_MAX_CONTENT_SLOTS; ++i)
    {
        // TODO: Move to setup. We need to release this in the discon.
        urb = usb_alloc_urb(0, GFP_ATOMIC);
        if (!urb)
        {
            break;
        }

        // TODO: Move to setup. We need to release this in the discon.
        buf = usb_alloc_coherent(dev->udev, dev->read_ep.max_packet_size, GFP_ATOMIC, &urb->transfer_dma);
        if (!buf)
        {
            netdev_err(netdev, "no memory left for USB buffer\n");
            usb_free_urb(urb);
            break;
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
            netdev_err(netdev, "usb_submit_urb failed: %d\n", retval);

            usb_unanchor_urb(urb);
            usb_free_coherent(dev->udev, dev->read_ep.max_packet_size, buf, urb->transfer_dma);
            usb_free_urb(urb);
            break;
        }

        dev->rxbuf[i] = buf;
        dev->rxbuf_dma[i] = urb->transfer_dma;

        /* Drop reference, USB core will take care of freeing it */
        usb_free_urb(urb);
    }

    /* If we did not submit any URBs then exit */
    if (i == 0)
    {
        netdev_err(netdev, "could not setup RX URBs\n");
        return retval;
    }

    /* Warn if we couldn't transmit all the URBs */
    if (i < RX_MAX_CONTENT_SLOTS)
    {
        netdev_warn(netdev, "clould not register all RX URBs\n");
    }

    return 0;
}

/*
 * Open the network device.
 *
 * This function is called when the network device transitions to the up state.
 * It will initialize the RX buffers and activate the TX queue.
 */
static int avic_can_open(struct net_device *netdev)
{
    int err = 0;

    netdev_info(netdev, "open device");

    err = open_candev(netdev);
    if (err)
    {
        return err;
    }

    can_led_event(netdev, CAN_LED_EVENT_OPEN);

    avic_can_netif_init(netdev);

    /* Accept packets on the network queue */
    netif_start_queue(netdev);

    return 0;
}

static void avic_can_netif_reset(struct net_device *netdev)
{
    struct avic_bridge *dev = netdev_priv(netdev);
    int i = 0;

    usb_kill_anchored_urbs(&dev->rx_submitted);

    for (i = 0; i < RX_MAX_CONTENT_SLOTS; ++i)
    {
        usb_free_coherent(dev->udev, dev->read_ep.max_packet_size,
                          dev->rxbuf[i], dev->rxbuf_dma[i]);
    }

    usb_kill_anchored_urbs(&dev->tx_submitted);
    atomic_set(&dev->tx_active, 0);
}

/*
 * Close the network device.
 *
 * This function is called when a network device transitions to the down state.
 * It will free resources and stop the TX queue.
 */
static int avic_can_close(struct net_device *netdev)
{
    netdev_info(netdev, "close device");

    /* We'll no longer accept new packets */
    netif_stop_queue(netdev);

    avic_can_netif_reset(netdev);

    close_candev(netdev);

    can_led_event(netdev, CAN_LED_EVENT_STOP);

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

/*
 * Probe USB device and see if we can claim the interface.
 *
 * If the interface is compatible with AVIC CAN then probe the for endpoints and
 * setup the network device.
 */
static int avic_usb_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct net_device *netdev = NULL;
    struct avic_bridge *dev = NULL;
    struct usb_endpoint_descriptor *read_in = NULL, *write_out = NULL;
    int retval = -ENOMEM;

    pr_info("found AVIC CAN interface\n");

    netdev = alloc_candev(sizeof(struct avic_bridge), 32);
    if (!netdev)
    {
        dev_err(&intf->dev, "could not allocate candev");
        return retval;
    }

    dev = netdev_priv(netdev);

    dev->udev = interface_to_usbdev(intf);
    dev->netdev = netdev;

    dev->can.state = CAN_STATE_STOPPED;
    dev->can.clock.freq = AVIC_USB_ABP_CLOCK;
    dev->can.bittiming_const = &avic_can_bittiming_const;
    dev->can.do_set_bittiming = avic_can_set_bittiming;
    dev->can.do_set_mode = avic_can_set_mode;
    dev->can.ctrlmode_supported = CAN_CTRLMODE_3_SAMPLES;

    atomic_set(&dev->tx_active, 0);
    init_usb_anchor(&dev->rx_submitted);
    init_usb_anchor(&dev->tx_submitted);

    netdev->netdev_ops = &avic_can_netdev_ops;

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
        dev_err(&intf->dev, "did not find bulk endpoints\n");
        goto cleanup_candev;
    }

    dev->write_ep.address = usb_endpoint_num(write_out);
    dev->write_ep.max_packet_size = usb_endpoint_maxp(write_out);

    if (dev->write_ep.max_packet_size < MIN_BULK_PACKET_SIZE)
    {
        dev_err(&intf->dev, "write bulk max size too small");

        retval = -ENODEV;
        goto cleanup_candev;
    }

    dev->read_ep.address = usb_endpoint_num(read_in);
    dev->read_ep.max_packet_size = usb_endpoint_maxp(read_in);

    if (dev->read_ep.max_packet_size < MIN_BULK_PACKET_SIZE)
    {
        dev_err(&intf->dev, "read bulk max size too small");

        retval = -ENODEV;
        goto cleanup_candev;
    }

    /* Entangle the USB interface with the AVIC bridge device */
    usb_set_intfdata(intf, dev);

    SET_NETDEV_DEV(netdev, &intf->dev);

    retval = register_candev(netdev);
    if (retval)
    {
        dev_err(&intf->dev, "could not register CAN device: %d\n", retval);

        goto cleanup_candev;
    }

    return 0;

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
