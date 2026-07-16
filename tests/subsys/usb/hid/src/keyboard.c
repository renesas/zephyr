/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/hid.h>
#include <zephyr/logging/log.h>
#include <zephyr/input/input.h>
#include <sample_usbd.h>
#include <zephyr/usb/class/usbd_hid.h>
#include <zephyr/usb/class/usbh_hid.h>

#include "usbh_desc.h"
#include "usbh_device.h"
#include "hid_keyboard.h"

LOG_MODULE_REGISTER(usbh_test_hid, LOG_LEVEL_INF);

#define MAX_EVENTS 16

struct usbh_hid_keyboard_suite_fixture {
	size_t expected_event_index;
	struct input_event expected_events[MAX_EVENTS];
	struct k_sem sync;
};

static struct device const *hid_dev = DEVICE_DT_GET(DT_NODELABEL(hid_keyboard));
static struct device const *usbh_hid_dev = NULL;
static struct usbd_context *test_usbd;

USBH_CONTROLLER_DEFINE(test_uhs_ctx, DEVICE_DT_GET(DT_NODELABEL(zephyr_uhc0)));

struct usbh_context *const uhs_ctx = &test_uhs_ctx;

static struct usbh_hid_keyboard_suite_fixture fixture = {};

static void *suite_setup(void)
{
	int result = 0;

	k_sem_init(&fixture.sync, 0, 1);

	result = usbh_init(uhs_ctx);
	zassert_ok(result, "Failed to initialize USB host");

	result = usbh_enable(uhs_ctx);
	zassert_ok(result, "Failed to enable USB host");

	result = uhc_bus_reset(uhs_ctx->dev);
	zassert_ok(result, "Failed to signal bus reset");

	result = uhc_bus_resume(uhs_ctx->dev);
	zassert_ok(result, "Failed to signal bus resume");

	result = uhc_sof_enable(uhs_ctx->dev);
	zassert_ok(result, "Failed to enable SoF generator");

	result = hid_keyboard_register();
	zassert_ok(result, "Failed to register HID device");

	test_usbd = sample_usbd_setup_device(NULL);
	zassert_not_null(test_usbd, "Failed to setup USB device");

	result = usbd_init(test_usbd);
	zassert_ok(result, "Failed to initialize device support");

	result = usbd_enable(test_usbd);
	zassert_ok(result, "Failed to enable device support");

	/* Allow the host time to reset the device. */
	k_msleep(500);

	usbh_hid_dev = device_get_binding("usbh_hid_0");
	zassert_not_null(usbh_hid_dev);
	result = usbh_hid_start_input_reports(usbh_hid_dev);
	zassert_ok(result, "Failed to start input reports");

	return NULL;
}

static void suite_shutdown(void *f)
{
	int result = 0;

	result = usbd_disable(test_usbd);
	zassert_ok(result, "Failed to disable device support");

	result = usbd_shutdown(test_usbd);
	zassert_ok(result, "Failed to shutdown device support");

	LOG_INF("Device support disabled");

	result = usbh_disable(uhs_ctx);
	zassert_ok(result, "Failed to disable USB host");

	result = usbh_shutdown(uhs_ctx);
	zassert_ok(result, "Failed to shutdown host support");

	LOG_INF("Host controller disabled");
}

ZTEST_SUITE(usbh_hid_keyboard_suite, NULL, suite_setup, NULL, NULL, suite_shutdown);

ZTEST(usbh_hid_keyboard_suite, test_usbh_hid_keyboard_input_single_keys)
{
	{
		/* X button press */
		uint8_t report[] = {0, 0, 0x1B, 0, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_X;
		fixture.expected_events[0].value = 1;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}

	{
		/* X button release */
		uint8_t report[] = {0, 0, 0, 0, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_X;
		fixture.expected_events[0].value = 0;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}

	{
		/* Left control modifier press */
		uint8_t report[] = {1, 0, 0, 0, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_LEFTCTRL;
		fixture.expected_events[0].value = 1;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}

	{
		/* Left control modifier release */
		uint8_t report[] = {0, 0, 0, 0, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_LEFTCTRL;
		fixture.expected_events[0].value = 0;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}
}

ZTEST(usbh_hid_keyboard_suite, test_usbh_hid_keyboard_input_multiple_keys)
{
	{
		/* Left control, X and Y press */
		uint8_t report[] = {1, 0, 0x1B, 0x1C, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_LEFTCTRL;
		fixture.expected_events[0].value = 1;

		fixture.expected_events[1].type = INPUT_EV_KEY;
		fixture.expected_events[1].code = INPUT_KEY_X;
		fixture.expected_events[1].value = 1;

		fixture.expected_events[2].type = INPUT_EV_KEY;
		fixture.expected_events[2].code = INPUT_KEY_Y;
		fixture.expected_events[2].value = 1;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}

	{
		/* X release */
		uint8_t report[] = {1, 0, 0x1C, 0, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_LEFTCTRL;
		fixture.expected_events[0].value = 1;

		fixture.expected_events[1].type = INPUT_EV_KEY;
		fixture.expected_events[1].code = INPUT_KEY_Y;
		fixture.expected_events[1].value = 1;

		fixture.expected_events[2].type = INPUT_EV_KEY;
		fixture.expected_events[2].code = INPUT_KEY_X;
		fixture.expected_events[2].value = 0;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}

	{
		/* Left control and Y release */
		uint8_t report[] = {0, 0, 0, 0, 0, 0, 0, 0};

		memset(fixture.expected_events, 0, sizeof(fixture.expected_events));
		fixture.expected_event_index = 0;
		fixture.expected_events[0].type = INPUT_EV_KEY;
		fixture.expected_events[0].code = INPUT_KEY_LEFTCTRL;
		fixture.expected_events[0].value = 0;

		fixture.expected_events[1].type = INPUT_EV_KEY;
		fixture.expected_events[1].code = INPUT_KEY_Y;
		fixture.expected_events[1].value = 0;

		hid_device_submit_report(hid_dev, sizeof(report), report);
		k_sem_take(&fixture.sync, K_FOREVER);
	}
}

ZTEST(usbh_hid_keyboard_suite, test_usbh_hid_keyboard_output_report)
{
	uint8_t const report[] = {0xF};
	usbh_hid_set_report(usbh_hid_dev, HID_REPORT_TYPE_OUTPUT, 0, sizeof(report), report);
}

static void verify_input_cb(struct input_event *evt, void *user_data)
{
	zassert_equal(fixture.expected_events[fixture.expected_event_index].type, evt->type,
		      "Wrong input event type");
	zassert_equal(fixture.expected_events[fixture.expected_event_index].code, evt->code,
		      "Wrong input event code");
	zassert_equal(fixture.expected_events[fixture.expected_event_index].value, evt->value,
		      "Wrong input event value");

	fixture.expected_event_index++;

	if (fixture.expected_events[fixture.expected_event_index].type == 0u) {
		k_sem_give(&fixture.sync);
	}
}

INPUT_CALLBACK_DEFINE(NULL, verify_input_cb, NULL);
