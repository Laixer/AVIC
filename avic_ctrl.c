// SPDX-License-Identifier: GPL-2.0
/*
 * AVIC control driver.
 *
 * Copyright (C) 2021 Yorick de Wid (yorick@laixer.com)
 * Copyright (C) 2021 Laixer Equipment B.V.
 */

// TODO:
// - reset

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/usb.h>

#include "avic.h"

/* Driver identifier */
#define DRV_NAME "avic_ctrl"

// TODO: This needs to be assigned by usb-dev mailing list.
#define AVIC_USB_CTRL_MINOR_BASE 192

#define CLOCK_SYNC_INTERVAL_SEC 60000

static struct usb_driver avic_usb_driver;

struct avic_usb_tx_urb_context
{
    struct avic_bridge *dev;
    unsigned int is_free;
    unsigned int index;
};

struct avic_control_bridge
{
    struct usb_device *udev;

    struct avic_endpoint status_ep;

    struct mutex io_mutex;
    bool is_reading;
    wait_queue_head_t is_reading_wait;
};

/* USB command requests. */
enum command_request_type
{
    /* Request info dump to console output. */
    USB_REQ_DUMP_INFO = 2,

    /* Clock synchronization request. */
    USB_REQ_SYNC_CLOCK = 7,

    /* System reset request. */
    USB_REQ_SYSTEM_RESET = 9,

    /* Subsystem reset request. */
    USB_REQ_SUBSYSTEM_RESET = 10,

    /* Halt all motor functions. */
    USB_REQ_HALT_MOTION = 128,
};

/* AVIC command request. */
struct command_request
{
    /* Command request. */
    enum command_request_type request;

    /* Optional value. */
    u16 value;
};

struct avic_timer_list
{
    struct timer_list timer;
    struct avic_control_bridge *dev;
};

static struct avic_timer_list clock_sync_timer;

static void avic_usb_fire_and_forget_callback(struct urb *urb)
{
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
}

/**
 * Submit control message without waiting for the result.
 * 
 * This method can be used to send control messages when the result is either not
 * important or the caller cannot wait for the result. Such a situation is common
 * in interrupt handlers and critical sections.
 */
int usb_control_msg_submit(struct usb_device *dev, unsigned int pipe, u8 request,
                           u8 request_type, u16 value, u16 index, void *data, u16 size)
{
    struct usb_ctrlrequest data_request;
    struct urb *urb = usb_alloc_urb(0, GFP_KERNEL);

    if (!urb)
    {
        return -ENOMEM;
    }

    data_request.bRequestType = request_type;
    data_request.bRequest = request;
    data_request.wValue = cpu_to_le16(value);
    data_request.wIndex = cpu_to_le16(index);
    data_request.wLength = cpu_to_le16(size);

    usb_fill_control_urb(urb, dev, pipe, (unsigned char *)&data_request,
                         data, size, avic_usb_fire_and_forget_callback, NULL);

    return usb_submit_urb(urb, GFP_KERNEL);
}
EXPORT_SYMBOL_GPL(usb_control_msg_submit);

static int command_request_send(struct avic_control_bridge *dev,
                                struct command_request *command)
{
    u8 *data = NULL;
    size_t data_sz = 0;
    int retval = 0;

    switch (command->request)
    {
    case USB_REQ_DUMP_INFO:
        command->value = 0;
        break;
    case USB_REQ_SYSTEM_RESET:
        command->value = 0;
        break;
    case USB_REQ_SUBSYSTEM_RESET:
        break;
    case USB_REQ_HALT_MOTION:
        command->value = 0;
        break;
    default:
        pr_warn("invalid command request\n");
        return -EINVAL;
    }

    retval = usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, AVIC_USB_CONTROL_ENDPOINT_ADDRESS),
                             command->request, USB_TYPE_CLASS | USB_RECIP_INTERFACE, command->value, 0,
                             data, data_sz, USB_CTRL_SET_TIMEOUT);
    if (unlikely(retval < 0))
    {
        pr_warn("usb_control_msg failed: %d\n", retval);

        return retval;
    }

    return 0;
}

static void avic_usb_read_interrupt_callback(struct urb *urb)
{
    struct avic_control_bridge *dev = urb->context;

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

    dev->is_reading = 0;
    wake_up_interruptible(&dev->is_reading_wait);
}

static int avic_ctrl_open(struct inode *inode, struct file *file)
{
    struct avic_control_bridge *dev = NULL;
    struct usb_interface *intf = NULL;
    int subminor = iminor(inode);
    int retval = 0;

    intf = usb_find_interface(&avic_usb_driver, subminor);
    if (!intf)
    {
        pr_err("cannot find device for minor: %d\n", subminor);

        retval = -ENODEV;
        goto exit;
    }

    dev = usb_get_intfdata(intf);
    if (!dev)
    {
        retval = -ENODEV;
        goto exit;
    }

    /* Point the file object to the device state so we can retrieve it later */
    file->private_data = dev;

exit:
    return retval;
}

static int avic_ctrl_release(struct inode *inode, struct file *file)
{
    struct avic_control_bridge *dev = file->private_data;

    if (!dev)
    {
        return -ENODEV;
    }

    file->private_data = NULL;

    return 0;
}

static ssize_t avic_ctrl_read(struct file *file, char *buffer, size_t count,
                              loff_t *ppos)
{
    struct avic_control_bridge *dev = file->private_data;
    int retval = 0;

    if (!dev)
    {
        return -ENODEV;
    }

    /* Sanity check */
    if (!count)
    {
        return 0;
    }

    /*
     * Accept a single reader at a time because there is only one
     * status URB. While waiting sustain interrupts.
     */
    retval = mutex_lock_interruptible(&dev->io_mutex);
    if (retval < 0)
    {
        return retval;
    }

    usb_fill_int_urb(dev->status_ep.urb, dev->udev,
                     usb_rcvintpipe(dev->udev, 1), // TODO: address
                     dev->status_ep.buffer,
                     dev->status_ep.info.max_packet_size,
                     avic_usb_read_interrupt_callback, dev, 1);

    retval = usb_submit_urb(dev->status_ep.urb, GFP_KERNEL);
    if (unlikely(retval))
    {
        pr_err("usb_submit_urb failed: %d\n", retval);

        goto exit;
    }

    dev->is_reading = 1;

    /*
     * Wait until the URB is finished. This can take significant time so we'll allow
     * interrupts in the meantime.
     */
    retval = wait_event_interruptible(dev->is_reading_wait, (!dev->is_reading));
    if (retval < 0)
    {
        goto exit;
    }

    if (copy_to_user(buffer, dev->status_ep.buffer, dev->status_ep.frame_sz))
    {
        retval = -EFAULT;
    }
    else
    {
        retval = dev->status_ep.frame_sz;
    }

exit:
    mutex_unlock(&dev->io_mutex);
    return retval;
}

static ssize_t avic_ctrl_write(struct file *file, const char *buffer,
                               size_t count, loff_t *ppos)
{
    struct avic_control_bridge *dev = file->private_data;
    u8 local_buffer[32];
    int retval = 0;

    if (!dev)
    {
        return -ENODEV;
    }

    /* Must have command buffer. */
    if (!count)
    {
        retval = -EINVAL;
        return retval;
    }

    /* Buffer overflow. */
    if (count > sizeof(local_buffer))
    {
        retval = -EINVAL;
        return retval;
    }

    if (copy_from_user(&local_buffer, buffer, count))
    {
        retval = -EFAULT;
        return retval;
        // goto error;
    }

    retval = command_request_send(dev, (struct command_request *)&local_buffer);
    if (unlikely(retval < 0))
    {
        return retval;
    }

    return count;
}

static const struct file_operations avic_ctrl_fops = {
    .owner = THIS_MODULE,
    .read = avic_ctrl_read,
    .write = avic_ctrl_write,
    .open = avic_ctrl_open,
    .release = avic_ctrl_release,
};

/**
 * USB class driver info in order to get a minor number from the USB core,
 * and to have the device registered with the driver core. The driver core
 * will then initialize the file operations and create the character device.
 */
static struct usb_class_driver avic_ctrl_class = {
    .name = "avicctrl%d",
    .fops = &avic_ctrl_fops,
    .minor_base = AVIC_USB_CTRL_MINOR_BASE,
};

/**
 * Synchronize the peripheral clock (wall clock) with the host.
 * 
 * Any time deviations will propagate downwards with degrading accuracy.
 * It is therefore assumed that the kernel time is accurate and correct.
 * 
 * This method can be called anytime and *must* be called no less than
 * once every clock hour to keep synchonized network time drift to a minimum.
 * 
 * This method will not block and can be safely called from interrupt context
 * or critical section. For the same reason it is undetermined if the command
 * was accepted.
 */
static int sync_peripheral_clock(struct avic_control_bridge *dev)
{
    ktime_t timestamp;
    int retval = 0;

    timestamp = ktime_get_real();

    retval = usb_control_msg_submit(dev->udev, usb_sndctrlpipe(dev->udev, AVIC_USB_CONTROL_ENDPOINT_ADDRESS),
                                    USB_REQ_SYNC_CLOCK, USB_TYPE_CLASS | USB_RECIP_INTERFACE, 0, 0,
                                    &timestamp, sizeof(ktime_t));
    if (unlikely(retval))
    {
        pr_err("usb_control_msg_submit failed: %d\n", retval);

        return retval;
    }

    pr_info("peripheral clock synchonization requested\n");

    return 0;
}

void clock_sync_timer_callback(struct timer_list *timer)
{
    struct avic_timer_list *local_clock_sync_timer = from_timer(&clock_sync_timer,
                                                                timer,
                                                                timer);

    sync_peripheral_clock(local_clock_sync_timer->dev);

    /* Reschedule the same task. */
    mod_timer(&clock_sync_timer.timer,
              jiffies + msecs_to_jiffies(CLOCK_SYNC_INTERVAL_SEC));
}

/**
 * Configure the peripheral before use. Add any future confuration or setup
 * commands to this method. The order of execution is of importance.
 */
static int avic_control_configure_peripheral(struct avic_control_bridge *dev)
{
    int retval = 0;
    struct command_request command_dump_info;

    /**
     * Dump the peripheral information to the log. This is usefull for 
     * diagnostics.
     * 
     * The standard demands this command to be send before any other
     * application level data is send on the out endpoint. Therefore
     * this command must be the first issued within this method.
     */
    command_dump_info.request = USB_REQ_DUMP_INFO;
    retval = command_request_send(dev, &command_dump_info);

    retval = sync_peripheral_clock(dev);

    /**
     * Start the interval timer which will synchonize the peripheral clock.
     * 
     * A low resolution jiffies timer is more than sufficient for the clock
     * synchronization operation which need not to be percise.
     */
    clock_sync_timer.dev = dev;
    timer_setup(&clock_sync_timer.timer, clock_sync_timer_callback, 0);
    mod_timer(&clock_sync_timer.timer, jiffies + msecs_to_jiffies(CLOCK_SYNC_INTERVAL_SEC));

    return retval;
}

static int avic_usb_probe(struct usb_interface *intf,
                          const struct usb_device_id *id)
{
    struct avic_control_bridge *dev = NULL;
    struct usb_endpoint_descriptor *status_in = NULL;
    int retval = -ENOMEM;

    pr_info("found AVIC control interface\n");

    /* Allocate device state with generic kernel priority */
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev)
    {
        pr_err("could not allocate device state");

        return retval;
    }

    dev->udev = interface_to_usbdev(intf);

    mutex_init(&dev->io_mutex);
    init_waitqueue_head(&dev->is_reading_wait);

    /*
     * Look for the interrupt status endpoint in the current interface
     * descriptor. This endpoint *must* exist otherwise we're uncertain
     * about the AVIC bridge configuration.
     */
    retval = usb_find_int_in_endpoint(intf->cur_altsetting, &status_in);
    if (retval)
    {
        pr_err("did not find status endpoint\n");

        goto cleanup_dev;
    }

    dev->status_ep.info.address = usb_endpoint_num(status_in);
    dev->status_ep.info.max_packet_size = usb_endpoint_maxp(status_in);

    dev->status_ep.urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!dev->status_ep.urb)
    {
        pr_err("could not allocate urb");

        retval = -ENOMEM;
        goto cleanup_dev;
    }

    dev->status_ep.buffer = kmalloc(dev->status_ep.info.max_packet_size, GFP_KERNEL);
    if (!dev->status_ep.buffer)
    {
        pr_err("could not allocate buffer");

        retval = -ENOMEM;
        goto cleanup_status_ep;
    }

    /* Entangle the USB interface with the AVIC bridge device */
    usb_set_intfdata(intf, dev);

    retval = usb_register_dev(intf, &avic_ctrl_class);
    if (retval)
    {
        pr_err("unable to get a minor for control class");

        goto cleanup_status_ep_buffer;
    }

    return avic_control_configure_peripheral(dev);

cleanup_status_ep_buffer:
    usb_set_intfdata(intf, NULL);
    kfree(dev->status_ep.buffer);

cleanup_status_ep:
    usb_free_urb(dev->status_ep.urb);

cleanup_dev:
    kfree(dev);

    return retval;
}

/*
 * Called by the usb core when the device is removed from the system.
 *
 * This method will release all unmanaged resources back to the kernel.
 */
static void avic_usb_disconnect(struct usb_interface *intf)
{
    struct avic_control_bridge *dev = usb_get_intfdata(intf);

    usb_set_intfdata(intf, NULL);

    if (dev)
    {
        del_timer(&clock_sync_timer.timer);

        /* Return minor to control class */
        usb_deregister_dev(intf, &avic_ctrl_class);

        usb_unlink_urb(dev->status_ep.urb);

        kfree(dev->status_ep.buffer);

        usb_free_urb(dev->status_ep.urb);

        kfree(dev);
    }
}

/* Table of devices that work with the AVIC bridge driver. */
static struct usb_device_id avic_usb_table[] = {
    {USB_DEVICE_AND_INTERFACE_INFO(AVIC_BRIDGE_VENDOR_ID,
                                   AVIC_BRIDGE_PRODUCT_ID,
                                   AVIC_BRIDGE_CAN_IFACE_CLASS,
                                   AVIC_BRIDGE_CAN_IFACE_SUBCLASS_CONTROL,
                                   AVIC_BRIDGE_CAN_IFACE_PROTO)},
    {} /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, avic_usb_table);

static ssize_t temperature_show(struct device *udev,
                                struct device_attribute *attr,
                                char *buf)
{
    struct usb_interface *intf = to_usb_interface(udev);
    struct avic_control_bridge *dev = usb_get_intfdata(intf);

    if (dev)
    {
        u32 *temp = (u32 *)&dev->status_ep.buffer[2];
        return sysfs_emit(buf, "%u\n", *temp);
    }

    return 0;
}

static DEVICE_ATTR_RO(temperature);

static struct attribute *dev_attrs[] = {
    &dev_attr_temperature.attr,
    NULL,
};

static struct attribute_group dev_attr_group = {
    .attrs = dev_attrs,
};

static const struct attribute_group *dev_attr_groups[] = {
    &dev_attr_group,
    NULL,
};

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
MODULE_DESCRIPTION("AVIC control bridge driver");
MODULE_LICENSE("GPL");
