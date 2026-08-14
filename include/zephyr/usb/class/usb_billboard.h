/*
 * Copyright (c) 2026 Renesas Electronics Corporation, Embedd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Device/host common USB Billboard header file
 */

#ifndef ZEPHYR_INCLUDE_USB_CLASS_USB_BILLBOARD_H
#define ZEPHYR_INCLUDE_USB_CLASS_USB_BILLBOARD_H

#include <stdint.h>
#include <zephyr/sys/util.h>

/**
 * @brief AUM part of Billboard capability descriptor
 */
struct usb_billboard_aum {
	uint16_t wSVID;
	uint8_t bAlternateOrUSB4Mode;
	uint8_t iAlternateOrUSB4ModeString;
} __packed;

/**
 * @brief Billboard capability descriptor with flexible AUM array
 */
struct usb_billboard_capability_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDevCapabilityType;
	uint8_t iAdditionalInfoURL;
	uint8_t bNumberOfAlternateOrUSB4Modes;
	uint8_t bPreferredAlternateOrUSB4Mode;
	uint16_t VCONNPower;
	uint8_t bmConfigured[32];
	uint16_t bcdVersion;
	uint8_t bAdditionalFailureInfo;
	uint8_t bReserved;
	FLEXIBLE_ARRAY_DECLARE(struct usb_billboard_aum, aum);
} __packed;

/**
 * @brief AUM capability descriptor
 */
struct usb_aum_capability_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint8_t bDevCapabilityType;
	uint8_t bIndex;
	uint32_t dwAlternateModeVdo;
} __packed;

#define USB_BILLBOARD_SUBCLASS 0x00
#define USB_BILLBOARD_RUNTIME  0x00

enum usb_billboard_vconn_needed {
	USB_BILLBOARD_VCONN_NEEDED_1W = 0,
	USB_BILLBOARD_VCONN_NEEDED_1W5 = 1,
	USB_BILLBOARD_VCONN_NEEDED_2W = 2,
	USB_BILLBOARD_VCONN_NEEDED_3W = 3,
	USB_BILLBOARD_VCONN_NEEDED_4W = 4,
	USB_BILLBOARD_VCONN_NEEDED_5W = 5,
	USB_BILLBOARD_VCONN_NEEDED_6W = 6,
	USB_BILLBOARD_VCONN_NEEDED_RESERVED = 7
};

#define USB_BILLBOARD_VCONN_GET_NEEDED(vconn_field) ((vconn_field) & 0x7)

#define USB_BILLBOARD_VCONN_NOT_NEEDED_MASK (1U << 15)
#define USBBILLBOARD__VCONN_NOT_NEEDED(vconn_field)                                                \
	(!!((vconn_field) & (USB_BILLBOARD_VCONN_NOT_NEEDED_MASK)))

enum usb_billboard_aum_state {
	USB_BILLBOARD_AUM_STATE_ERROR = 0,
	USB_BILLBOARD_AUM_STATE_NOT_ATTEMPTED = 1,
	USB_BILLBOARD_AUM_STATE_NOT_ATTEMPTED_FAILED = 2,
	USB_BILLBOARD_AUM_STATE_SUCCESSFUL = 3
};

#define USB_BILLBOARD_GET_AUM_INDEX(alt_idx) ((alt_idx) / 4)
#define USB_BILLBOARD_GET_AUM_BITPOS(aum_elem, alt_idx)                                            \
	(((aum_elem) >> (((alt_idx) % 4) * 2)) & 0x3)
#define USB_BILLBOARD_GET_AUM(bmConfigured, alt_idx)                                               \
	USB_BILLBOARD_GET_AUM_BITPOS((bmConfigured)[USB_BILLBOARD_GET_AUM_INDEX(alt_idx)], alt_idx)

#endif
