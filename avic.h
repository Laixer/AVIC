// SPDX-License-Identifier: GPL-2.0
/*
 * AVIC interface.
 *
 * Copyright (C) 2021 Yorick de Wid (yorick@laixer.com)
 * Copyright (C) 2021 Laixer Equipment B.V.
 * 
 * AVIC or Advanced Vehicle InterConnect is an open protocol to
 * communicate between a host controller and a peripheral. The
 * protocol necessitates a reliable data link layer such as USB.
 */

/* USB vendor and product identifiers */
#define AVIC_BRIDGE_VENDOR_ID 0x1d6b
#define AVIC_BRIDGE_PRODUCT_ID 0x27dd

/* USB CAN interface properties */
#define AVIC_BRIDGE_CAN_IFACE_CLASS 0xfe
#define AVIC_BRIDGE_CAN_IFACE_SUBCLASS_CONTROL 0x4
#define AVIC_BRIDGE_CAN_IFACE_SUBCLASS_DATA 0xc
#define AVIC_BRIDGE_CAN_IFACE_PROTO 0

/* Peripheral-side endpoint info. */
struct avic_usb_endpoint_info
{
    /* Endpoint address. */
    int address;

    /* Maximum packet size for this pipe. */
    int max_packet_size;
};

/* Endpoint data pipe structure. */
struct avic_endpoint
{
    /* The endpoint descriptor. */
    struct avic_usb_endpoint_info info;

    /* USB Request Block. */
    struct urb *urb;

    /* Data written to or read from pipe. */
    u8 *buffer;

    /**
     * Actual data length in buffer. This is smaller than or eqaul
     * to info.max_packet_size.
     */
    int frame_sz;
};
