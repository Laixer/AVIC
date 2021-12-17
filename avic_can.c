// SPDX-License-Identifier: GPL-2.0
/*
 * AVIC Bridge driver.
 *
 * Copyright (C) 2021 Yorick de Wid (yorick@laixer.com)
 * Copyright (C) 2021 Laixer Equipment B.V.
 * 
 * This driver includes the following mini drivers:
 *  - CAN FD USB proxy via can-dev module
 *  - AVIC controller
 */

// TODO:
// - More read buffers
// - Locking

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>

#include <linux/can.h>
#include <linux/can/dev.h>

/* Driver identifier */
#define DRV_NAME "avic_can"

/* USB vendor and product identifiers */
#define AVIC_BRIDGE_VENDOR_ID 0x1d6b
#define AVIC_BRIDGE_PRODUCT_ID 0x27dd

/* USB CAN interface properties */
#define AVIC_BRIDGE_CAN_IFACE_CLASS 255
#define AVIC_BRIDGE_CAN_IFACE_SUBCLASS 0
#define AVIC_BRIDGE_CAN_IFACE_PROTO 0

/* USB interrupt requests */
#define USB_REQ_SYNC_CLOCK 7    /* Synchornize the host clock */
#define USB_REQ_SYSTEM_RESET 9  /* Request a reset on the peripheral */
#define USB_REQ_HALT_MOTION 128 /* Halt all motion devices connected to the peripheral */

/* AVIC frame default port. Can change in the future */
#define AVIC_PORT_DEFAULT 0

/* AVIC frame types */
#define AVIC_FRAME_TYPE_CAN 0x8       /* CAN bus frame */
#define AVIC_FRAME_TYPE_CAN_FD 0x9    /* CAN FD bus frame */
#define AVIC_FRAME_TYPE_ETHERNET 0x11 /* Ethernet frame */

#define TX_MAX_CONTENT_SLOTS 32
#define RX_MAX_CONTENT_SLOTS 32

#define MIN_BULK_PACKET_SIZE 64

struct avic_usb_endpoint_info
{
    int address;
    int max_packet_size;
};

// TODO: include info struct
struct avic_endpoint
{
    int address;

    struct urb *urb;
    u8 *buffer;
    int buffer_sz;

    int frame_sz;
};

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

    struct avic_endpoint status_ep;

    struct avic_usb_endpoint_info write_ep;
    struct avic_usb_endpoint_info read_ep;

    struct avic_usb_tx_urb_context tx_context[TX_MAX_CONTENT_SLOTS];
};

struct avic_frame
{
    __u8 type;     /* Frame type */
    __u32 id;      /* Source address */
    __u8 port;     /* AVIC port */
    __u8 len;      /* Payload length */
    __u8 data[48]; /* Payload */
} __attribute__((packed));

static struct avic_bridge *g_dev = NULL;

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

    can_get_echo_skb(netdev, context->index, NULL);

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

    if (frame->can_id & CAN_RTR_FLAG)
    {
        // avic_frame->flags |= USB_8DEV_RTR;
    }

    if (frame->can_id & CAN_EFF_FLAG)
    {
        // avic_frame->flags |= USB_8DEV_EXTID;
    }

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

        netdev->stats.tx_dropped++;
        return NETDEV_TX_BUSY;
    }

    usb_fill_bulk_urb(urb, dev->udev, usb_sndbulkpipe(dev->udev, dev->write_ep.address),
                      buf, dev->write_ep.max_packet_size,
                      avic_usb_write_bulk_callback, context);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    usb_anchor_urb(urb, &dev->tx_submitted);

    can_put_echo_skb(skb, netdev, context->index, 0);

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
    int err = 0;

    err = open_candev(netdev);
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
    // struct avic_bridge *dev = netdev_priv(netdev);

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
    return 0;
}

static void avic_usb_read_interrupt_callback(struct urb *urb)
{
    struct avic_bridge *dev = urb->context;
    int retval;

    dev->status_ep.frame_sz = urb->actual_length;

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
        pr_warn("rx interrupt aborted: %d\n", urb->status);
        return;
    }

    /* Resubmit self */
    retval = usb_submit_urb(urb, GFP_KERNEL);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);
    }
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

    frame->can_id = avic_frame->id;
    frame->len = avic_frame->len;
    memcpy(frame->data, avic_frame->data, avic_frame->len);

    /* At this point the receive was a success so update the stats */
    netdev->stats.rx_packets++;
    netdev->stats.rx_bytes += urb->actual_length;

    netif_rx(skb);

    usb_fill_bulk_urb(urb, dev->udev,
                      usb_rcvbulkpipe(dev->udev, dev->read_ep.address),
                      urb->transfer_buffer, dev->read_ep.max_packet_size,
                      avic_usb_read_bulk_callback, dev);

    retval = usb_submit_urb(urb, GFP_KERNEL);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);
    }
}

static int avic_usb_configure_peripheral(struct avic_bridge *dev)
{
    struct urb *urb = NULL;
    u8 *buf = NULL;
    int retval = -ENOMEM;

    pr_info("synchonize peripheral clock with host\n");

    /* Clock sync */
    retval = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0),
                             USB_REQ_SYNC_CLOCK, USB_TYPE_CLASS | USB_RECIP_INTERFACE, 0x3, 0,
                             NULL, 0, USB_CTRL_SET_TIMEOUT);
    if (unlikely(retval))
    {
        pr_warn("usb_control_msg failed: %d\n", retval);

        return retval;
    }

    /* Bootstrap the status interrupt reader */
    usb_fill_int_urb(dev->status_ep.urb, dev->udev,
                     usb_rcvintpipe(dev->udev, 1),
                     dev->status_ep.buffer,
                     dev->status_ep.buffer_sz,
                     avic_usb_read_interrupt_callback, dev, 1);

    retval = usb_submit_urb(dev->status_ep.urb, GFP_KERNEL);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);

        return retval;
    }

    pr_info("status endpoint interval running\n");

    urb = usb_alloc_urb(0, GFP_ATOMIC);
    if (!urb)
    {
        return retval;
    }

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

static int avic_can_setup(struct usb_interface *intf, const struct usb_device_id *id)
{
    struct net_device *netdev;
    struct avic_bridge *dev;
    struct usb_endpoint_descriptor *read_in, *write_out, *status_in;
    int i = 0, retval = -ENOMEM;

    pr_info("found AVIC CAN interface\n");

    netdev = alloc_candev(sizeof(struct avic_bridge), 32);
    if (!netdev)
    {
        pr_err("could not alloc candev");
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
     * Look for the interrupt status endpoint in the current interface
     * descriptor. This endpoint *must* exist otherwise we're uncertain
     * about the AVIC bridge configuration.
     */
    retval = usb_find_common_endpoints(intf->cur_altsetting,
                                       &read_in,
                                       &write_out,
                                       &status_in, NULL);
    if (retval)
    {
        pr_err("did not find expected endpoints\n");
        goto cleanup_candev;
    }

    dev->status_ep.address = usb_endpoint_num(status_in);
    dev->status_ep.buffer_sz = usb_endpoint_maxp(status_in);

    dev->status_ep.urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!dev->status_ep.urb)
    {
        pr_err("Couldn't alloc intr_urb");

        retval = -ENOMEM;
        goto cleanup_candev;
    }

    dev->status_ep.buffer = kmalloc(dev->status_ep.buffer_sz, GFP_KERNEL);
    if (!dev->status_ep.buffer)
    {
        pr_err("Couldn't alloc intr_in_buffer");

        retval = -ENOMEM;
        goto cleanup_status_ep;
    }

    dev->write_ep.address = usb_endpoint_num(write_out);
    dev->write_ep.max_packet_size = usb_endpoint_maxp(write_out);

    if (dev->write_ep.max_packet_size < MIN_BULK_PACKET_SIZE)
    {
        pr_err("write bulk max size too small");

        retval = -ENODEV;
        goto cleanup_status_ep_buffer;
    }

    dev->read_ep.address = usb_endpoint_num(read_in);
    dev->read_ep.max_packet_size = usb_endpoint_maxp(read_in);

    if (dev->read_ep.max_packet_size < MIN_BULK_PACKET_SIZE)
    {
        pr_err("read bulk max size too small");

        retval = -ENODEV;
        goto cleanup_status_ep_buffer;
    }

    /* Entangle the USB interface with the AVIC bridge device */
    usb_set_intfdata(intf, dev);

    SET_NETDEV_DEV(netdev, &intf->dev);

    retval = register_candev(netdev);
    if (retval)
    {
        pr_err("could not register CAN device: %d\n", retval);
        goto cleanup_status_ep_buffer;
    }

    g_dev = dev;

    return avic_usb_configure_peripheral(dev);

cleanup_status_ep_buffer:
    kfree(dev->status_ep.buffer);

cleanup_status_ep:
    usb_free_urb(dev->status_ep.urb);

cleanup_candev:
    free_candev(netdev);

    return retval;
}

/*
 * Probe the USB interface and determine which AVIC controller will
 * be loaded.
 */
static int avic_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
    return avic_can_setup(intf, id);
}

/*
 * called by the usb core when the device is removed from the system
 */
static void avic_usb_disconnect(struct usb_interface *intf)
{
    struct avic_bridge *dev = usb_get_intfdata(intf);

    usb_set_intfdata(intf, NULL);

    if (dev)
    {
        usb_unlink_urb(dev->status_ep.urb);

        kfree(dev->status_ep.buffer);

        usb_free_urb(dev->status_ep.urb);

        unregister_candev(dev->netdev);

        free_candev(dev->netdev);

        g_dev = NULL;
    }
}

/* Table of devices that work with the AVIC bridge driver. */
static struct usb_device_id avic_usb_table[] = {
    {USB_DEVICE_AND_INTERFACE_INFO(AVIC_BRIDGE_VENDOR_ID, AVIC_BRIDGE_PRODUCT_ID, AVIC_BRIDGE_CAN_IFACE_CLASS, AVIC_BRIDGE_CAN_IFACE_SUBCLASS, AVIC_BRIDGE_CAN_IFACE_PROTO)},
    {} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, avic_usb_table);

static ssize_t temperature_show(struct kobject *kobj, struct kobj_attribute *attr,
                                char *buf)
{
    // TODO: Get rid of the global. There must be a way to fetch the corresponding device.
    if (g_dev)
    {
        u32 *temp = (u32 *)&g_dev->status_ep.buffer[2];
        return sysfs_emit(buf, "%u\n", *temp);
    }

    // TODO: Maybe an error?
    return sysfs_emit(buf, "%u\n", 0);
}

static struct kobj_attribute chl_temperature_attr = __ATTR_RO(temperature);

static struct attribute *dev_attrs[] = {
    &chl_temperature_attr.attr,
    NULL,
};

static struct attribute_group dev_attr_group = {
    .attrs = dev_attrs,
};

static const struct attribute_group *dev_attr_groups[] = {
    &dev_attr_group,
    NULL,
};

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver avic_usb_driver = {
    .name = DRV_NAME,
    .probe = avic_usb_probe,
    .disconnect = avic_usb_disconnect,
    .id_table = avic_usb_table,
    .dev_groups = dev_attr_groups,
};

module_usb_driver(avic_usb_driver);

MODULE_AUTHOR("Yorick de Wid <yorick@laixer.com>");
MODULE_AUTHOR("Laixer Equipment B.V.");
MODULE_DESCRIPTION("AVIC Bridge driver");
MODULE_LICENSE("GPL");
