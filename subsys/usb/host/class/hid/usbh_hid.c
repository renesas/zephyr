/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_usbh_hid_device

#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/input/input.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/usb/class/usbh_hid.h>

#include "usbh_class.h"
#include "usbh_desc.h"
#include "usbh_ch9.h"
#include "usbh_device.h"

LOG_MODULE_REGISTER(usbh_hid, CONFIG_USBH_HID_LOG_LEVEL);

#define KEYBOARD_BOOT_PROTOCOL_MODIFIER_INDEX 0u
#define KEYBOARD_BOOT_PROTOCOL_KEY_INDEX      2u
#define KEYBOARD_BOOT_PROTOCOL_KEYS           6u
#define KEYBOARD_BOOT_PROTOCOL_REPORT_SIZE    8u
#define KEYBOARD_BOOT_PROTOCOL_MODIFIERS      8u
#define MOUSE_BOOT_PROTOCOL_BUTTONS_INDEX     0u
#define MOUSE_BOOT_PROTOCOL_X_INDEX           1u
#define MOUSE_BOOT_PROTOCOL_Y_INDEX           2u
#define MOUSE_BOOT_PROTOCOL_REPORT_SIZE       3u
#define MOUSE_BOOT_PROTOCOL_BUTTONS           3u

struct driver_config {
	/* Size of the idle rates array */
	size_t idle_rates_ms_size;
	/* Idle period in milliseconds */
	uint16_t idle_rates_ms[CONFIG_USBH_HID_REPORT_MAX_VARIANTS * 2];
	/* Whether this device starts with boot protocol */
	bool start_with_boot_protocol;
};

struct driver_data {
	/* USB host class device */
	struct device const *dev;
	/* Connected usb device */
	struct usb_device *udev;
	/* Mutual exclusion lock */
	struct k_mutex lock;
	/* Sync semaphore for waiting on asynchronous operations */
	struct k_sem sync;
	/* Interrupt IN endpoint address */
	struct usb_ep_descriptor const *input_ep;
	/* Interrupt OUT endpoint address */
	struct usb_ep_descriptor const *output_ep;
	/* Interface index */
	uint8_t target_iface;
	/* Report descriptor */
	struct usbh_hid_report report;
	/* Interrupt report request */
	struct uhc_transfer *interrupt_in_xfer;
	/* Whether the interrupt report request is in progress */
	bool input_interrupt_report_on;
	/* Whether the currently configured protocol is the simplified boot protocol */
	bool boot_protocol;
#if CONFIG_USBH_HID_ROUTE_TO_INPUT
	/* Previous input reports */
	struct {
		uint8_t id;
		uint8_t data[CONFIG_USBH_HID_MAX_INPUT_REPORT_SIZE];
	} previous_reports[CONFIG_USBH_HID_REPORT_MAX_VARIANTS];
#endif
	/* Callback to be invoked with input data */
	usbh_hid_report_cb_t input_cb;
	/* User data */
	void *user_data;
};

static int req_interrupt_input(struct device const *dev);
static int driver_stop_input_reports(struct device const *dev);
static int driver_set_idle_rate(struct device const *dev, uint8_t report_id,
				uint16_t idle_period_ms);
static int driver_set_protocol(struct device const *dev, uint8_t protocol_code);

/*
 * Check whether the connected device is a keyboard
 */
static inline bool is_keyboard(struct driver_data *const driver_data)
{
	struct usb_if_descriptor const *interface =
		usbh_desc_get_iface(driver_data->udev, driver_data->target_iface);
	return interface->bInterfaceProtocol == HID_BOOT_IFACE_CODE_KEYBOARD;
}

/*
 * Check whether the connected device is a mouse
 */
static inline bool is_mouse(struct driver_data *const driver_data)
{
	struct usb_if_descriptor const *interface =
		usbh_desc_get_iface(driver_data->udev, driver_data->target_iface);
	return interface->bInterfaceProtocol == HID_BOOT_IFACE_CODE_MOUSE;
}

#if CONFIG_USBH_HID_ROUTE_TO_INPUT
/**
 * Conversion map between the HID key usage IDs and Zephyr's input keys
 */
static const uint16_t hid_key_to_input_map[] = {
	INPUT_KEY_A,          INPUT_KEY_B,          INPUT_KEY_C,         INPUT_KEY_D,
	INPUT_KEY_E,          INPUT_KEY_F,          INPUT_KEY_G,         INPUT_KEY_H,
	INPUT_KEY_I,          INPUT_KEY_J,          INPUT_KEY_K,         INPUT_KEY_L,
	INPUT_KEY_M,          INPUT_KEY_N,          INPUT_KEY_O,         INPUT_KEY_P,
	INPUT_KEY_Q,          INPUT_KEY_R,          INPUT_KEY_S,         INPUT_KEY_T,
	INPUT_KEY_U,          INPUT_KEY_V,          INPUT_KEY_W,         INPUT_KEY_X,
	INPUT_KEY_Y,          INPUT_KEY_Z,          INPUT_KEY_1,         INPUT_KEY_2,
	INPUT_KEY_3,          INPUT_KEY_4,          INPUT_KEY_5,         INPUT_KEY_6,
	INPUT_KEY_7,          INPUT_KEY_8,          INPUT_KEY_9,         INPUT_KEY_0,
	INPUT_KEY_ENTER,      INPUT_KEY_ESC,        INPUT_KEY_BACKSPACE, INPUT_KEY_TAB,
	INPUT_KEY_SPACE,      INPUT_KEY_MINUS,      INPUT_KEY_EQUAL,     INPUT_KEY_RESERVED,
	INPUT_KEY_RESERVED,   INPUT_KEY_BACKSLASH,  INPUT_KEY_RESERVED,  INPUT_KEY_SEMICOLON,
	INPUT_KEY_APOSTROPHE, INPUT_KEY_GRAVE,      INPUT_KEY_COMMA,     INPUT_KEY_DOT,
	INPUT_KEY_SLASH,      INPUT_KEY_CAPSLOCK,   INPUT_KEY_F1,        INPUT_KEY_F2,
	INPUT_KEY_F3,         INPUT_KEY_F4,         INPUT_KEY_F5,        INPUT_KEY_F6,
	INPUT_KEY_F7,         INPUT_KEY_F8,         INPUT_KEY_F9,        INPUT_KEY_F10,
	INPUT_KEY_F11,        INPUT_KEY_F12,        INPUT_KEY_PRINT,     INPUT_KEY_SCROLLLOCK,
	INPUT_KEY_PAUSE,      INPUT_KEY_INSERT,     INPUT_KEY_HOME,      INPUT_KEY_PAGEUP,
	INPUT_KEY_DELETE,     INPUT_KEY_END,        INPUT_KEY_PAGEDOWN,  INPUT_KEY_RIGHT,
	INPUT_KEY_LEFT,       INPUT_KEY_DOWN,       INPUT_KEY_UP,        INPUT_KEY_NUMLOCK,
	INPUT_KEY_KPSLASH,    INPUT_KEY_KPASTERISK, INPUT_KEY_KPMINUS,   INPUT_KEY_KPPLUS,
	INPUT_KEY_KPENTER,    INPUT_KEY_KP1,        INPUT_KEY_KP2,       INPUT_KEY_KP3,
	INPUT_KEY_KP4,        INPUT_KEY_KP5,        INPUT_KEY_KP6,       INPUT_KEY_KP7,
	INPUT_KEY_KP8,        INPUT_KEY_KP9,        INPUT_KEY_KP0,       INPUT_KEY_KPDOT,
};

/**
 * Conversion map between the HID modifier key usage IDs and Zephyr's input keys
 */
static const uint16_t hid_modifier_to_input_map[] = {
	INPUT_KEY_LEFTCTRL,  INPUT_KEY_LEFTSHIFT,  INPUT_KEY_LEFTALT,  INPUT_KEY_LEFTMETA,
	INPUT_KEY_RIGHTCTRL, INPUT_KEY_RIGHTSHIFT, INPUT_KEY_RIGHTALT, INPUT_KEY_RIGHTMETA,
};

/**
 * Conversion map between the HID button usage IDs and Zephyr's input buttons
 */
static const uint16_t hid_button_to_input_map[] = {
	INPUT_BTN_0, INPUT_BTN_1, INPUT_BTN_2, INPUT_BTN_3, INPUT_BTN_4,
	INPUT_BTN_5, INPUT_BTN_6, INPUT_BTN_7, INPUT_BTN_8, INPUT_BTN_9,
};

/*
 * Forward an HID key or button press to the input subsystem
 */
static void report_event(struct device const *dev, size_t map_length,
			 uint16_t const map[map_length], uint16_t map_base, uint8_t usage_id,
			 bool value, bool sync)
{

	size_t input_index = 0;
	if (usage_id < map_base) {
		/* Out of range */
		return;
	}

	input_index = usage_id - map_base;
	if (input_index >= map_length) {
		/* Out of range */
		return;
	}

	if (map[input_index] == INPUT_KEY_RESERVED) {
		/* To be ignored */
		return;
	}

	input_report_key(dev, map[input_index], value, sync, K_FOREVER);
}

/*
 * Forward an HID key press to the input subsystem
 */
static void report_key(struct device const *dev, uint8_t usage_id, bool value, bool sync)
{
	report_event(dev, sizeof(hid_key_to_input_map) / sizeof(hid_key_to_input_map[0]),
		     hid_key_to_input_map, HID_KEY_A, usage_id, value, sync);
}

/*
 * Forward an HID modifier key press to the input subsystem
 */
static void report_modifier_key(struct device const *dev, uint8_t usage_id, bool value, bool sync)
{
	report_event(dev, sizeof(hid_modifier_to_input_map) / sizeof(hid_modifier_to_input_map[0]),
		     hid_modifier_to_input_map, HID_USAGE_GEN_DESKTOP_KEYBOARD_LEFT_CTRL, usage_id,
		     value, sync);
}

/*
 * Forward an HID button press to the input subsystem
 */
static void report_button(struct device const *dev, uint8_t usage_id, bool value, bool sync)
{
	report_event(dev, sizeof(hid_button_to_input_map) / sizeof(hid_button_to_input_map[0]),
		     hid_button_to_input_map, HID_BTN_1, usage_id, value, sync);
}

/*
 * Get a pointer to the buffer where the previous report is stored
 * returns -ENOMEM if there are no buffers avaialble for the provided report ID
 */
static int get_previous_report_buffer(struct driver_data const *driver_data, uint8_t report_id)
{
	int result = -ENOMEM;
	struct usbh_hid_report const *report = &driver_data->report;

	/* Check if the current report actually has multiple variants */
	if (report->num_reports > 1u) {
		for (size_t report_index = 0u; report_index < report->num_reports; report_index++) {
			if (driver_data->previous_reports[report_index].id == report_id ||
			    driver_data->previous_reports[report_index].id == 0) {
				result = report_index;
				break;
			}
		}
	} else {
		result = 0;
	}

	return result;
}

/*
 * Checks whether a key was previously pressed and now not anymore
 */
static bool was_array_key_released(uint8_t const *prev_value_ptr, uint8_t const *value_ptr,
				   size_t prev_key_index, size_t count)
{
	bool released = false;

	/* The key was previously pressed, so now it may be released */
	if (prev_value_ptr[prev_key_index] != 0u) {
		bool found = false;

		for (size_t key_index = 0u; key_index < count; key_index++) {
			/* Key is still pressed */
			if (value_ptr[key_index] == prev_value_ptr[prev_key_index]) {
				found = true;
				break;
			}
		}

		/* If not found, it is released */
		released = !found;
	}

	return released;
}

/*
 * Report keys from an array report field
 */
static void report_array_keys(struct device const *dev, uint8_t const *prev_value_ptr,
			      uint8_t const *value_ptr, size_t bit_shift, size_t count)
{

	/* Active keys */
	for (size_t key_index = 0u; key_index < count; key_index++) {
		if (value_ptr[key_index] != 0u) {
			report_key(dev, value_ptr[key_index], true, false);
		}
	}

	/* Inactive keys */
	for (size_t prev_key_index = 0u; prev_key_index < count; prev_key_index++) {
		if (was_array_key_released(prev_value_ptr, value_ptr, prev_key_index, count)) {
			report_key(dev, prev_value_ptr[prev_key_index], false, false);
		}
	}
}

/*
 * Report a key from a variable report field
 */
static void report_variable_key(struct device const *dev, uint8_t const *prev_value_ptr,
				uint8_t const *value_ptr, size_t key_position, uint16_t usage_id)
{
	/* Pinpoint the exact bit */
	size_t key_index = key_position / 8u;
	size_t key_shift = key_position % 8u;

	/* Active key */
	if ((value_ptr[key_index] & (1u << key_shift)) > 0u) {
		report_modifier_key(dev, usage_id, true, false);
	}
	/* Inactive key */
	else if ((prev_value_ptr[key_index] & (1u << key_shift)) > 0u) {
		report_modifier_key(dev, usage_id, false, false);
	}
}

/*
 * Report a button from a variable report field
 */
static void report_variable_button(struct device const *dev, uint8_t const *prev_value_ptr,
				   uint8_t const *value_ptr, size_t button_position,
				   uint16_t usage_id)
{
	/* Pinpoint the exact bit */
	size_t button_index = button_position / 8u;
	size_t button_shift = button_position % 8u;

	/* Active buttons */
	if ((value_ptr[button_index] & (1u << button_shift)) > 0u) {
		report_button(dev, usage_id, true, false);
	}
	/* Inactive buttons */
	else if ((prev_value_ptr[button_index] & (1u << button_shift)) > 0u) {
		report_button(dev, usage_id, false, false);
	}
}

/*
 * Report events from a report field
 */
static int report_events(struct usbh_hid_report_field const *field, uint8_t report_id,
			 uint8_t const *data, size_t bit_index, void *user_data)
{
	size_t value_index = bit_index / 8;
	size_t value_bit_shift = bit_index % 8;
	int result = 0;
	struct driver_data *const driver_data = user_data;
	struct device const *dev = driver_data->dev;
	uint8_t const *value_ptr = &data[value_index];
	uint8_t const *prev_value_ptr = NULL;
	int32_t signed_value = 0u;

	result = get_previous_report_buffer(driver_data, report_id);
	if (result < 0) {
		LOG_WRN("Unable to get previous buffer for report ID 0x%02X", report_id);
		return result;
	}

	prev_value_ptr = &driver_data->previous_reports[result].data[value_index];

	/* Keyboard input */
	if (usbh_hid_report_match_usage_page(field, HID_USAGE_GEN_KEYBOARD)) {
		/* Keyboard array, each element is a button press */
		if (USBH_HID_REPORT_DATA_IS_ARRAY(field->flags) && field->size == 8u) {
			report_array_keys(dev, prev_value_ptr, value_ptr, value_bit_shift,
					  field->count);
		}
		/* Keyboard variable, mostly for modifiers */
		else if (USBH_HID_REPORT_DATA_IS_VARIABLE(field->flags) && field->size == 1u) {
			for (size_t key_position = 0u; key_position < field->count;
			     key_position++) {
				report_variable_key(dev, prev_value_ptr, value_ptr,
						    value_bit_shift + key_position,
						    usbh_hid_report_field_get_usage_id_by_index(
							    field, key_position));
			}
		}
	}
	/* Mouse buttons */
	else if (usbh_hid_report_match_usage_page(field, HID_USAGE_GEN_BUTTON)) {
		if (USBH_HID_REPORT_DATA_IS_VARIABLE(field->flags) && field->size == 1u) {
			for (size_t button_position = 0u; button_position < field->count;
			     button_position++) {
				report_variable_button(dev, prev_value_ptr, value_ptr,
						       value_bit_shift + button_position,
						       usbh_hid_report_field_get_usage_id_by_index(
							       field, button_position));
			}
		}
	} else {
		/* Mouse relative X movement */
		if (USBH_HID_REPORT_DATA_IS_RELATIVE(field->flags) &&
		    usbh_hid_report_get_usage_id_i32(field, value_ptr, value_bit_shift,
						     (HID_USAGE_GEN_DESKTOP << 16u) |
							     HID_USAGE_GEN_DESKTOP_X,
						     &signed_value) == 0) {
			if (signed_value != 0u) {
				input_report_rel(dev, INPUT_REL_X, signed_value, false, K_FOREVER);
			}
		}
		/* Mouse relative Y movement */
		if (USBH_HID_REPORT_DATA_IS_RELATIVE(field->flags) &&
		    usbh_hid_report_get_usage_id_i32(field, value_ptr, value_bit_shift,
						     (HID_USAGE_GEN_DESKTOP << 16) |
							     HID_USAGE_GEN_DESKTOP_Y,
						     &signed_value) == 0) {
			if (signed_value != 0u) {
				input_report_rel(dev, INPUT_REL_Y, signed_value, false, K_FOREVER);
			}
		}
		/* Mouse relative wheel movement */
		if (usbh_hid_report_get_usage_id_i32(field, value_ptr, value_bit_shift,
						     (HID_USAGE_GEN_DESKTOP << 16u) |
							     HID_USAGE_GEN_DESKTOP_WHEEL,
						     &signed_value) == 0) {
			if (signed_value != 0u) {
				input_report_rel(dev, INPUT_REL_WHEEL, signed_value, false,
						 K_FOREVER);
			}
		}
	}

	return 0;
}
#endif

/*
 * Manage an input report
 *
 */
static int handle_input_report(struct driver_config const *driver_config,
			       struct driver_data *driver_data, size_t data_length,
			       uint8_t const data[data_length])
{
#if CONFIG_USBH_HID_ROUTE_TO_INPUT
	size_t input_size = 0u;
	size_t previous_report_index = 0u;
	uint8_t report_id = 0u;
	struct device const *dev = driver_data->dev;
	int result = 0;
#endif

	LOG_HEXDUMP_DBG(data, data_length, "RX  : ");

#if CONFIG_USBH_HID_ROUTE_TO_INPUT
	if (driver_data->boot_protocol) {
		report_id = 0;

		/* Mouse boot protocol */
		if (is_mouse(driver_data)) {
			int8_t x = 0;
			int8_t y = 0;

			/* Not enough data */
			if (data_length < MOUSE_BOOT_PROTOCOL_REPORT_SIZE) {
				return -EIO;
			}

			x = data[MOUSE_BOOT_PROTOCOL_X_INDEX];
			y = data[MOUSE_BOOT_PROTOCOL_Y_INDEX];
			input_size = MOUSE_BOOT_PROTOCOL_REPORT_SIZE;

			/* Keyboard variable, mostly for modifiers */
			for (size_t key_position = 0u; key_position < MOUSE_BOOT_PROTOCOL_BUTTONS;
			     key_position++) {
				report_variable_button(
					dev,
					&driver_data->previous_reports[0u]
						 .data[MOUSE_BOOT_PROTOCOL_BUTTONS_INDEX],
					&data[MOUSE_BOOT_PROTOCOL_BUTTONS_INDEX], key_position,
					HID_BTN_1 + key_position);
			}

			if (x != 0) {
				input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
			}
			if (y != 0) {
				input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
			}
		}
		/* Keyboard boot protocol */
		else if (is_keyboard(driver_data)) {
			/* Not enough data */
			if (data_length < KEYBOARD_BOOT_PROTOCOL_REPORT_SIZE) {
				return -EIO;
			}

			input_size = KEYBOARD_BOOT_PROTOCOL_REPORT_SIZE;
			/* Keyboard array, each element is a button press */
			report_array_keys(dev,
					  &driver_data->previous_reports[0u]
						   .data[KEYBOARD_BOOT_PROTOCOL_KEY_INDEX],
					  &data[KEYBOARD_BOOT_PROTOCOL_KEY_INDEX], 0u,
					  KEYBOARD_BOOT_PROTOCOL_KEYS);

			/* Keyboard variable, mostly for modifiers */
			for (size_t key_position = 0u;
			     key_position < KEYBOARD_BOOT_PROTOCOL_MODIFIERS; key_position++) {
				report_variable_key(
					dev,
					&driver_data->previous_reports[0u]
						 .data[KEYBOARD_BOOT_PROTOCOL_MODIFIER_INDEX],
					&data[KEYBOARD_BOOT_PROTOCOL_MODIFIER_INDEX], key_position,
					HID_USAGE_GEN_DESKTOP_KEYBOARD_LEFT_CTRL + key_position);
			}
		}
	}
	/* Report dynamic parsing and handling */
	else {
		/* If multiple reports are specified pick the first byte of the data as the ID */
		if (driver_data->report.num_reports > 1u) {
			report_id = data[0u];
		}

		result = get_previous_report_buffer(driver_data, report_id);
		if (result < 0) {
			LOG_WRN("Unable to get previous buffer for report ID 0x%02X", report_id);
			return result;
		}
		previous_report_index = result;

		input_size = usbh_hid_report_get_input_size(&driver_data->report, report_id);

		if (input_size == data_length) {
			usbh_hid_report_input_iterate(&driver_data->report, data_length, data,
						      report_events, driver_data);
		} else {
			LOG_WRN("Length mismatch between report descriptor "
				"and actual report: %i vs %i",
				input_size, data_length);
		}
	}

	/* Update the previous report buffer */
	memcpy(&driver_data->previous_reports[previous_report_index].data, data, input_size);
	driver_data->previous_reports[previous_report_index].id = report_id;
#endif

	/* If specified invoke the user provided callback */
	if (driver_data->input_cb) {
		usbh_hid_report_input_iterate(&driver_data->report, data_length, data,
					      driver_data->input_cb, driver_data->user_data);
	}

	return 0;
}

/*
 * Clear an endpoint, re-enabling it
 */
static int clear_feature_endpoint_halt(struct driver_data *driver_data, uint8_t endpoint)
{
	/* See USB specification, section 9.4 */
	const uint8_t bmRequestType = (USB_REQTYPE_DIR_TO_DEVICE << 7u) |
				      (USB_REQTYPE_TYPE_STANDARD << 5u) |
				      (USB_REQTYPE_RECIPIENT_ENDPOINT << 0u);
	return usbh_req_setup(driver_data->udev, bmRequestType, USB_SREQ_CLEAR_FEATURE,
			      USB_SFS_ENDPOINT_HALT, endpoint, 0, NULL);
}

/*
 * Callback on completion of input interrupt requests
 */
static int input_interrupt_transfer_cb(struct usb_device *const udev,
				       struct uhc_transfer *const xfer)
{
	struct device *const dev = xfer->priv;
	struct driver_data *driver_data = dev->data;
	struct driver_config const *driver_config = dev->config;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (xfer->err == 0) {
		/* The net_buf contains the received data */
		struct net_buf *buf = xfer->buf;
		if (USB_EP_DIR_IS_IN(xfer->ep)) {
			if (buf && buf->len > 0u) {
				result = handle_input_report(driver_config, driver_data, buf->len,
							     buf->data);
				if (result != 0) {
					LOG_WRN("Error handling input: %i", result);
				}
			}
		}
	}
	/* Endpoint halt condition to be cleared */
	else if (xfer->err == -EPIPE) {
		LOG_DBG("Endpoint in stalled, clearing and retrying...");
		result = clear_feature_endpoint_halt(driver_data,
						     driver_data->input_ep->bEndpointAddress);
		if (result != 0) {
			LOG_ERR("Could not restore input endpoint: %i", result);
			goto error_cleanup;
		}
	}
	/* Request was cancelled, nothing to do */
	else if (xfer->err == -ECONNRESET) {
	}
	/* Continue with the error */
	else {
		LOG_WRN("IN endpoint request failed: %i", xfer->err);
		result = xfer->err;
	}

	/* Clear the saved xfer reference */
	driver_data->interrupt_in_xfer = NULL;

	if (result == 0 && driver_data->input_interrupt_report_on) {
		result = req_interrupt_input(dev);
	} else {
		LOG_INF("Stopping input requests");
	}

error_cleanup:
	/* Finally free the request */
	if (xfer->buf != NULL) {
		usbh_xfer_buf_free(driver_data->udev, xfer->buf);
	}
	usbh_xfer_free(driver_data->udev, xfer);

	k_mutex_unlock(&driver_data->lock);

	return result;
}

/*
 * Enqueue an interrupt input request, kickstarting a periodic update
 */
static int req_interrupt_input(struct device const *dev)
{
	struct uhc_transfer *xfer = NULL;
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;

	/* Allocate with enough buffer for one interrupt packet */
	xfer = usbh_xfer_alloc_with_buf(driver_data->udev, driver_data->input_ep->bEndpointAddress,
					input_interrupt_transfer_cb, (void *)dev,
					driver_data->input_ep->wMaxPacketSize);
	if (xfer == NULL) {
		LOG_WRN("Unable to allocate memory buffer");
		return -ENOMEM;
	}

	/* Enqueue the transfer */
	result = usbh_xfer_enqueue(driver_data->udev, xfer);
	if (result != 0) {
		usbh_xfer_free(driver_data->udev, xfer);
		LOG_WRN("Unable to enqueue interrupt input request: %i", result);
		return result;
	}

	driver_data->interrupt_in_xfer = xfer;

	return 0;
}

/*
 * Enqueue a request for an interface descriptor
 */
static int req_iface_desc(struct usb_device *const udev, uint8_t const type, uint8_t const index,
			  uint16_t const id, uint16_t const len, struct net_buf *const buf)
{
	const uint8_t bmRequestType =
		(USB_REQTYPE_DIR_TO_HOST << 7u) | (USB_REQTYPE_RECIPIENT_INTERFACE << 0u);
	const uint8_t bRequest = USB_SREQ_GET_DESCRIPTOR;
	const uint16_t wValue = (type << 8u) | index;

	return usbh_req_setup(udev, bmRequestType, bRequest, wValue, id, len, buf);
}

/*
 * Enqueue a HID class request
 */
static int hid_class_request(struct usb_device *const udev, uint8_t const iface,
			     uint8_t const direction, uint8_t const request, uint16_t const value,
			     size_t data_length, struct net_buf *buf)
{
	const uint8_t bmRequestType = (direction << 7u) | (USB_REQTYPE_TYPE_CLASS << 5u) |
				      (USB_REQTYPE_RECIPIENT_INTERFACE << 0u);

	return usbh_req_setup(udev, bmRequestType, request, value, iface, data_length, buf);
}

/*
 * Analyze the various descriptors of the USB device interface
 */
static int scan_descriptors(struct driver_config const *driver_config,
			    struct driver_data *const driver_data, uint8_t const iface)
{
	int result = 0;
	struct net_buf *buf = NULL;
	struct usb_desc_header const *dhp = NULL;
	uint16_t report_descriptor_length = 0u;

	/*
	 * Fetch the size of the HID descriptor first by traversing the descriptors from the
	 * selected interface
	 */
	dhp = (struct usb_desc_header const *)usbh_desc_get_iface(driver_data->udev, iface);
	if (dhp == NULL) {
		LOG_ERR("Failed to find interface %u", iface);
		return -ENOSYS;
	}
	dhp = usbh_desc_get_next(dhp);

	while (dhp != NULL && dhp->bDescriptorType != USB_DESC_INTERFACE) {
		/* Report descriptor */
		if (dhp->bDescriptorType == USB_DESC_HID) {
			uint8_t const *descriptor_data = (uint8_t const *)dhp;
			/* Fetch the report descriptor length */
			report_descriptor_length = sys_get_le16(&descriptor_data[7u]);
		}
		/* Interrupt in endpoint */
		else if (dhp->bDescriptorType == USB_DESC_ENDPOINT) {
			struct usb_ep_descriptor const *const ep =
				(struct usb_ep_descriptor const *)dhp;
			/* Note down the interrupt IN endpoint */
			if (USB_EP_DIR_IS_IN(ep->bEndpointAddress) &&
			    (ep->bmAttributes & USB_EP_TRANSFER_TYPE_MASK) ==
				    USB_EP_TYPE_INTERRUPT) {
				LOG_INF("Found input endpoint at 0x%02X", ep->bEndpointAddress);
				driver_data->input_ep = ep;
			}
			/* Optional interrupt OUT endpoint for output and feature reports */
			else {
				LOG_INF("Found output endpoint at 0x%02X", ep->bEndpointAddress);
				driver_data->output_ep = ep;
			}
		}

		dhp = usbh_desc_get_next(dhp);
	}

	if (driver_data->input_ep == NULL) {
		LOG_ERR("No input endpoint found");
		return -ENOSYS;
	}

	/* Fetch report descriptor */
	if (report_descriptor_length == 0) {
		LOG_ERR("No report descriptor found");
		return -ENOSYS;
	}

	buf = usbh_xfer_buf_alloc(driver_data->udev, report_descriptor_length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	result = req_iface_desc(driver_data->udev, USB_DESC_HID_REPORT, 0u, iface,
				report_descriptor_length, buf);
	if (result != 0) {
		LOG_WRN("Request for HID report descriptor failed: %i", result);
		goto error_cleanup;
	}

	LOG_HEXDUMP_INF(buf->data, buf->len, "Report: ");

	result = usbh_hid_report_parse(&driver_data->report, buf->len, buf->data);
	if (result != 0) {
		LOG_WRN("Parsing of the HID report descriptor failed: %i", result);
		goto error_cleanup;
	}

error_cleanup:
	usbh_xfer_buf_free(driver_data->udev, buf);

	return result;
}

/*
 * Initialize the HID host class driver
 */
static int usbh_class_init(struct usbh_class_data *const c_data)
{
	struct device const *dev = c_data->priv;
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;

	LOG_INF("Initializing HID host data");

	memset(driver_data, 0x00, sizeof(*driver_data));

	driver_data->dev = dev;

	result = k_mutex_init(&driver_data->lock);
	if (result != 0) {
		return result;
	}

	result = k_sem_init(&driver_data->sync, 0, 1);
	if (result != 0) {
		return result;
	}

	LOG_INF("HID host data initialized successfully");
	return 0;
}

/*
 * Probe the USB class driver after a device has been found
 */
static int usbh_class_probe(struct usbh_class_data *const c_data, struct usb_device *const udev,
			    uint8_t iface)
{
	struct device const *dev = c_data->priv;
	struct driver_data *driver_data = dev->data;
	struct driver_config const *driver_config = dev->config;
	uint8_t target_iface = 0;
	int result = 0;

	if ((udev == NULL) || (udev->state != USB_STATE_CONFIGURED)) {
		LOG_ERR("USB device not properly configured");
		return -ENODEV;
	}

	if (driver_data == NULL) {
		LOG_ERR("No HID device instance available");
		return -ENODEV;
	}

	/* Convert device-level match to interface 0 */
	if (iface == USBH_CLASS_IFNUM_DEVICE) {
		target_iface = 0u;
	} else {
		target_iface = iface;
	}

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	/* Clear some data before starting */
	driver_data->input_ep = NULL;
	driver_data->output_ep = NULL;
	memset(&driver_data->report, 0, sizeof(driver_data->report));

	driver_data->udev = udev;
	driver_data->target_iface = target_iface;

	/* Fetch the relevant information */
	result = scan_descriptors(driver_config, driver_data, target_iface);
	if (result != 0) {
		goto error_cleanup;
	}

	/* Select the boot protocol */
	if (driver_config->start_with_boot_protocol) {
		result = driver_set_protocol(dev, HID_PROTOCOL_BOOT);
		if (result != 0) {
			LOG_ERR("Failed to set boot protocol: %i", result);
			goto error_cleanup;
		}
	}
	/* Set the generic protocol */
	else {
		result = driver_set_protocol(dev, HID_PROTOCOL_REPORT);
		if (result != 0) {
			LOG_ERR("Failed to set report protocol: %i", result);
			goto error_cleanup;
		}
	}

	/* Set the idle rate */
	for (size_t idle_rate_index = 0; idle_rate_index < driver_config->idle_rates_ms_size / 2;
	     idle_rate_index++) {
		uint8_t report_id = driver_config->idle_rates_ms[idle_rate_index * 2u];
		uint16_t idle_period_ms = driver_config->idle_rates_ms[idle_rate_index * 2u + 1u];

		result = driver_set_idle_rate(dev, report_id, idle_period_ms);
		if (result == -EPIPE) {
			/* Set idle not supported, can ignore */
		}
		/* Other error */
		else if (result != 0) {
			LOG_WRN("Failed to set idle rate for report ID 0x%02X: %i", report_id,
				result);
			goto error_cleanup;
		}
	}

	LOG_INF("HID device (addr=%d) initialization completed for iface %i",
		driver_data->udev->addr, target_iface);

	k_mutex_unlock(&driver_data->lock);

	return 0;

error_cleanup:
	k_mutex_unlock(&driver_data->lock);

	return result;
}

/*
 * Remove the USB class driver on disconnection
 */
static int usbh_class_remove(struct usbh_class_data *const c_data)
{
	struct device const *dev = c_data->priv;
	struct driver_data *driver_data = (void *)dev->data;

	driver_stop_input_reports(dev);

	k_mutex_lock(&driver_data->lock, K_FOREVER);

#if CONFIG_USBH_HID_ROUTE_TO_INPUT
	memset(driver_data->previous_reports, 0u, sizeof(driver_data->previous_reports));
#endif

	memset(&driver_data->report, 0u, sizeof(driver_data->report));

	driver_data->input_ep = NULL;
	driver_data->output_ep = NULL;
	driver_data->interrupt_in_xfer = NULL;
	driver_data->input_interrupt_report_on = false;
	driver_data->target_iface = 0;
	driver_data->udev = NULL;

	k_mutex_unlock(&driver_data->lock);

	LOG_INF("HID device removal completed");

	return 0;
}

/*
 * USB Host class API vtable
 */
static __maybe_unused struct usbh_class_api usbh_class_api = {
	.init = usbh_class_init,
	.probe = usbh_class_probe,
	.removed = usbh_class_remove,
};

/*
 * USB Host class filters
 */
static __maybe_unused struct usbh_class_filter const generic_hid_filters[] = {
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = USB_HID_SUBCLASS_NONE,
		.proto = HID_BOOT_IFACE_CODE_NONE,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = USB_HID_SUBCLASS_NONE,
		.proto = HID_BOOT_IFACE_CODE_MOUSE,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = USB_HID_SUBCLASS_BOOT,
		.proto = HID_BOOT_IFACE_CODE_MOUSE,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = USB_HID_SUBCLASS_BOOT,
		.proto = HID_BOOT_IFACE_CODE_KEYBOARD,
	},
	{
		.flags = USBH_CLASS_MATCH_CODE_TRIPLE,
		.class = USB_BCC_HID,
		.sub = USB_HID_SUBCLASS_NONE,
		.proto = HID_BOOT_IFACE_CODE_KEYBOARD,
	},
	{0u},
};

/*
 * Synchronization callback to wait for completion of asynchronous transfers
 * Should be passed to `usbh_xfer_alloc_with_buf` or `usbh_xfer_alloc` before queuing the
 * transfer, to then block on `driver_data->sync` in order to wait for completion. This
 * function only gives way to the semaphore; it doesn't analyze or deallocate anything.
 */
static int sync_cb(struct usb_device *const udev, struct uhc_transfer *const xfer)
{
	struct driver_data *driver_data = xfer->priv;

	if (xfer->err != 0) {
		LOG_DBG("Request finished %p, err %d, sem %i", xfer, xfer->err,
			k_sem_count_get(&driver_data->sync));
	}
	/* If the transfer was cancelled we deallocate it here */
	else if (xfer->err == -ECONNRESET) {
		LOG_INF("Transfer %p cancelled", (void *)xfer);
		usbh_xfer_free(udev, xfer);

		return 0;
	}

	k_sem_give(&driver_data->sync);

	return 0;
}

/*
 * Block on `driver_data->sync` waiting for the `sync_cb` callback
 * This function waits for the last transfer enqueued with `sync_cb` as completion to be
 * done. If it actually completes it returns the error code; in the event of a timeout it
 * makes sure the transfer is no longer pending.
 */
static int wait_for_sync(struct driver_data *driver_data, struct uhc_transfer *xfer)
{
	int result = 0;

	if (k_sem_take(&driver_data->sync, K_MSEC(100u)) != 0) {
		LOG_ERR("Timeout");

		result = usbh_xfer_dequeue(driver_data->udev, xfer);
		/* While the semaphore take timed out, the transfer was actually already
		 * done and the callback on its way. */
		if (result == -EALREADY) {
			/* Take the semaphore again to be sure that the callback is done */
			if (k_sem_take(&driver_data->sync, K_MSEC(100u)) != 0) {
				/* Should not happen */
				LOG_ERR("Double timeout");
			}
		}
		/* Dequeue failed */
		else if (result != 0) {
			LOG_ERR("Failed to cancel transfer");
		}
		/* Dequeue succeeded, do nothing */
		else {
		}

		/* The USB host driver may still need to work with the transfer, so we leave it to
		 * the callback to deallocate it.
		 */

		return -ETIMEDOUT;
	} else {
		/* The transfer was successful, store the result and free it */
		result = xfer->err;
		usbh_xfer_free(driver_data->udev, xfer);
	}

	return result;
}

static int driver_get_report_descriptor(struct device const *dev, struct usbh_hid_report *report)
{
	struct driver_data *driver_data = (void *)dev->data;

	if (report == NULL) {
		return -EINVAL;
	}

	/* Copy the report descriptor */
	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		k_mutex_unlock(&driver_data->lock);
		return -ENOTCONN;
	}

	memcpy(report, &driver_data->report, sizeof(struct usbh_hid_report));
	k_mutex_unlock(&driver_data->lock);

	return 0;
}

static int driver_get_report(struct device const *dev, enum usbh_hid_report_field_type type,
			     uint8_t report_id, size_t length, uint8_t *buffer)
{
	struct driver_data *driver_data = (void *)dev->data;
	struct net_buf *buf = NULL;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);
	buf = usbh_xfer_buf_alloc(driver_data->udev, length);
	if (buf == NULL) {
		k_mutex_unlock(&driver_data->lock);
		return -ENOMEM;
	}

	result = hid_class_request(driver_data->udev, driver_data->target_iface,
				   USB_REQTYPE_DIR_TO_HOST, USB_HID_GET_REPORT,
				   (type << 8u) | report_id, length, buf);

	if (result == 0) {
		memcpy(buffer, buf->data, buf->len);
	}

	usbh_xfer_buf_free(driver_data->udev, buf);
	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_set_report(struct device const *dev, enum usbh_hid_report_field_type type,
			     uint8_t report_id, size_t data_length, uint8_t const data[data_length])
{
	struct driver_data *driver_data = (void *)dev->data;
	struct net_buf *buf = NULL;
	int result = 0;

	if (data == NULL || data_length == 0u) {
		return -EINVAL;
	}

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		k_mutex_unlock(&driver_data->lock);
		return -ENOTCONN;
	}

	/* If an interrupt OUT endpoint is specified use that */
	if (driver_data->output_ep) {
		size_t buffer_length = data_length;

		/* If through the output interrupt and with a non null ID the report ID must be the
		 * first byte */
		if (report_id != 0) {
			buffer_length++;
		}

		buf = usbh_xfer_buf_alloc(driver_data->udev, buffer_length);
		if (buf == NULL) {
			result = -ENOMEM;
			goto error_cleanup;
		}

		if (report_id != 0) {
			net_buf_add_u8(buf, report_id);
		}
		net_buf_add_mem(buf, data, data_length);

		struct uhc_transfer *xfer =
			usbh_xfer_alloc(driver_data->udev, driver_data->output_ep->bEndpointAddress,
					sync_cb, driver_data);
		if (xfer == NULL) {
			result = -ENOMEM;
			goto error_cleanup;
		}

		result = usbh_xfer_buf_add(driver_data->udev, xfer, buf);
		if (result != 0) {
			usbh_xfer_free(driver_data->udev, xfer);
			goto error_cleanup;
		}

		LOG_DBG("Sending report ID 0x%02X on output endpoint", report_id);
		result = usbh_xfer_enqueue(driver_data->udev, xfer);
		if (result != 0) {
			usbh_xfer_free(driver_data->udev, xfer);
			goto error_cleanup;
		}

		/* Wait for completion, deallocation handled automatically */
		result = wait_for_sync(driver_data, xfer);
	}
	/* No interrupt OUT endpoint, fall back to a control request */
	else {
		buf = usbh_xfer_buf_alloc(driver_data->udev, data_length);
		if (buf == NULL) {
			result = -ENOMEM;
			goto error_cleanup;
		}

		net_buf_add_mem(buf, data, data_length);

		LOG_DBG("Sending report ID 0x%02X on control endpoint", report_id);
		result = hid_class_request(driver_data->udev, driver_data->target_iface,
					   USB_REQTYPE_DIR_TO_DEVICE, USB_HID_SET_REPORT,
					   ((uint16_t)type << 8u) | report_id, data_length, buf);
	}

error_cleanup:
	if (buf != NULL) {
		usbh_xfer_buf_free(driver_data->udev, buf);
	}

	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_start_input_reports(struct device const *dev)
{
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		k_mutex_unlock(&driver_data->lock);
		return -ENOTCONN;
	}

	if (driver_data->input_interrupt_report_on) {
		/* Process already started */
		k_mutex_unlock(&driver_data->lock);
		return -EINPROGRESS;
	}

	driver_data->input_interrupt_report_on = true;
	/* Start input inteerrupt */
	result = req_interrupt_input(dev);

	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_stop_input_reports(struct device const *dev)
{
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		k_mutex_unlock(&driver_data->lock);
		return -ENOTCONN;
	}

	if (!driver_data->input_interrupt_report_on) {
		/* Process not started */
		k_mutex_unlock(&driver_data->lock);
		return -ENOTCONN;
	}

	driver_data->input_interrupt_report_on = false;
	result = usbh_xfer_dequeue(driver_data->udev, driver_data->interrupt_in_xfer);

	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_set_input_callback(struct device const *dev, usbh_hid_report_cb_t callback,
				     void *user_data)
{
	struct driver_data *driver_data = (void *)dev->data;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	driver_data->input_cb = callback;
	driver_data->user_data = user_data;

	k_mutex_unlock(&driver_data->lock);

	return 0;
}

static bool has_report_id(struct usbh_hid_report const *report, uint8_t report_id)
{
	/* 0 means all report IDs; always present */
	if (report_id == 0) {
		return true;
	}
	/* If a specific report ID is given check if it actually matches one in the report
	 * descriptor */
	else {
		bool found = false;

		for (size_t report_index = 0; report_index < report->num_reports; report_index++) {
			if (report->reports[report_index].id == report_id) {
				found = true;
				break;
			}
		}

		return found;
	}
}

static int driver_set_idle_rate(struct device const *dev, uint8_t report_id,
				uint16_t idle_period_ms)
{
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		result = -ENOTCONN;
		goto error_cleanup;
	}

	if (!has_report_id(&driver_data->report, report_id)) {
		LOG_ERR("Report ID 0x%02X doesn't exist", report_id);
		result = -EINVAL;
		goto error_cleanup;
	}

	result = hid_class_request(driver_data->udev, driver_data->target_iface,
				   USB_REQTYPE_DIR_TO_DEVICE, USB_HID_SET_IDLE,
				   ((idle_period_ms / 4u) << 8u) | report_id, 0, NULL);
	/* Set idle not supported, can ignore */
	if (result == -EPIPE) {
		LOG_INF("HID does not support SET_IDLE");
		goto error_cleanup;
	}
	/* Any other error is a problem */
	else if (result != 0) {
		LOG_WRN("Failed to set idle rate: %i", result);
		goto error_cleanup;
	}

error_cleanup:
	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_get_idle_rate(struct device const *dev, uint8_t report_id,
				uint16_t *idle_period_ms)
{
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;
	struct net_buf *buf = NULL;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		result = -ENOTCONN;
		goto error_cleanup;
	}

	if (!has_report_id(&driver_data->report, report_id)) {
		LOG_ERR("Report ID 0x%02X doesn't exist", report_id);
		result = -EINVAL;
		goto error_cleanup;
	}

	buf = usbh_xfer_buf_alloc(driver_data->udev, 1);
	if (buf == NULL) {
		result = -ENOMEM;
		goto error_cleanup;
	}
	result = hid_class_request(driver_data->udev, driver_data->target_iface,
				   USB_REQTYPE_DIR_TO_HOST, USB_HID_GET_IDLE, report_id, 1, buf);
	/* Set idle not supported, can ignore */
	if (result == -EPIPE) {
		LOG_INF("HID does not support GET_IDLE");
		goto error_cleanup;
	}
	/* Any other error is a problem */
	else if (result != 0) {
		LOG_WRN("Failed to get idle rate: %i", result);
		goto error_cleanup;
	}

	*idle_period_ms = buf->data[0] * 4;

error_cleanup:
	if (buf != NULL) {
		usbh_xfer_buf_free(driver_data->udev, buf);
	}

	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_set_protocol(struct device const *dev, uint8_t protocol_code)
{
	struct driver_data *driver_data = (void *)dev->data;
	int result = 0;

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		result = -ENOTCONN;
		goto error_cleanup;
	}

	if (protocol_code == HID_PROTOCOL_BOOT) {
		struct usb_if_descriptor const *interface =
			usbh_desc_get_iface(driver_data->udev, driver_data->target_iface);
		/* The device may not support the boot protocol, in which case we abort */
		if (interface->bInterfaceSubClass != USB_HID_SUBCLASS_BOOT) {
			LOG_WRN("Device doesn't support boot protocol!");
			result = -ENOTSUP;
			goto error_cleanup;
		}
	}

	/* Set the requested protocol */
	result = hid_class_request(driver_data->udev, driver_data->target_iface,
				   USB_REQTYPE_DIR_TO_DEVICE, USB_HID_SET_PROTOCOL, protocol_code,
				   0, NULL);
	if (result != 0) {
		LOG_ERR("Failed to set protocol %i: %i", protocol_code, result);
		goto error_cleanup;
	}

	/* Save which protocol is currently enabled for the input event subsystem */
	if (protocol_code == HID_PROTOCOL_BOOT) {
		driver_data->boot_protocol = true;
	} else {
		driver_data->boot_protocol = false;
	}

error_cleanup:
	k_mutex_unlock(&driver_data->lock);

	return result;
}

static int driver_get_protocol(struct device const *dev, uint8_t *protocol_code)
{
	struct driver_data *driver_data = (void *)dev->data;
	struct net_buf *buf = NULL;
	int result = 0;

	if (protocol_code == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&driver_data->lock, K_FOREVER);

	if (driver_data->udev == NULL) {
		/* Device not yet connected */
		goto error_cleanup;
		return -ENOTCONN;
	}

	buf = usbh_xfer_buf_alloc(driver_data->udev, 1);
	if (buf == NULL) {
		k_mutex_unlock(&driver_data->lock);
		return -ENOMEM;
	}

	/* Set the requested protocol */
	result = hid_class_request(driver_data->udev, driver_data->target_iface,
				   USB_REQTYPE_DIR_TO_HOST, USB_HID_GET_PROTOCOL, 0, 1, buf);
	if (result != 0) {
		LOG_ERR("Failed to get protocol: %i", result);
		goto error_cleanup;
	}

	*protocol_code = buf->data[0];

error_cleanup:
	if (buf != NULL) {
		usbh_xfer_buf_free(driver_data->udev, buf);
	}

	k_mutex_unlock(&driver_data->lock);

	return result;
}

/*
 * USB Host HID driver API vtable
 */
DEVICE_API(usbh_hid, driver_api) = {
	.get_report_descriptor = driver_get_report_descriptor,
	.get_report = driver_get_report,
	.set_report = driver_set_report,
	.start_input_reports = driver_start_input_reports,
	.stop_input_reports = driver_stop_input_reports,
	.set_idle_rate = driver_set_idle_rate,
	.get_idle_rate = driver_get_idle_rate,
	.set_protocol = driver_set_protocol,
	.get_protocol = driver_get_protocol,
	.set_input_callback = driver_set_input_callback,
};

#define USBH_DEVICE_DEFINE(index)                                                                    \
	static struct driver_data driver_data_##index = {0u};                                        \
	static struct driver_config const driver_config_##index = {                                  \
		.idle_rates_ms = DT_INST_PROP(index, in_idle_rate_ms),                               \
		.idle_rates_ms_size = DT_INST_PROP_LEN(index, in_idle_rate_ms),                      \
		.start_with_boot_protocol = DT_INST_PROP(index, start_with_boot_protocol),         \
	};                                                                                           \
                                                                                                     \
	DEVICE_DT_INST_DEFINE(index, NULL, NULL, &driver_data_##index, &driver_config_##index,       \
			      POST_KERNEL, 50, &driver_api);                                         \
                                                                                                     \
	COND_CODE_0(DT_INST_PROP(index, match_class),                                              \
	(static struct usbh_class_filter const hid_filters_##index[] = {                           \
		{                                                                                  \
			.flags = USBH_CLASS_MATCH_VID_PID,                                         \
			.vid = (DT_INST_REG_ADDR(index) >> 16u) & 0xFFFFu,                         \
			.pid = DT_INST_REG_ADDR(index) & 0xFFFFu,                                  \
		},                                                                                 \
		{0u},                                                                              \
	};), ()                                                                                    \
	) \
                                                                                                     \
	USBH_DEFINE_CLASS(usbh_hid_data_##index, &usbh_class_api,                                    \
			  (void *)DEVICE_DT_INST_GET(index),                                         \
			  COND_CODE_1(DT_INST_PROP(index, match_class), (generic_hid_filters), (hid_filters_##index)));

DT_INST_FOREACH_STATUS_OKAY(USBH_DEVICE_DEFINE)
