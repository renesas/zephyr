/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/class/usbd_hid.h>

#include "hid_keyboard.h"

static const uint8_t hid_report_desc[] = HID_KEYBOARD_REPORT_DESC();

static uint32_t kb_duration;

static void kb_iface_ready(const struct device *dev, const bool ready)
{
}

static int kb_get_report(const struct device *dev, const uint8_t type, const uint8_t id,
			 const uint16_t len, uint8_t *const buf)
{
	return 0;
}

static int kb_set_report(const struct device *dev, const uint8_t type, const uint8_t id,
			 const uint16_t len, const uint8_t *const buf)
{
	if (type != HID_REPORT_TYPE_OUTPUT) {
		return -ENOTSUP;
	}

	return 0;
}

/* Idle duration is stored but not used to calculate idle reports. */
static void kb_set_idle(const struct device *dev, const uint8_t id, const uint32_t duration)
{
	kb_duration = duration;
}

static uint32_t kb_get_idle(const struct device *dev, const uint8_t id)
{
	return kb_duration;
}

static void kb_set_protocol(const struct device *dev, const uint8_t proto)
{
}

static void kb_output_report(const struct device *dev, const uint16_t len, const uint8_t *const buf)
{
	zassert_equal(1, len, "Wrong report length");
	zassert_equal(0xF, buf[0], "Wrong report value");
	kb_set_report(dev, HID_REPORT_TYPE_OUTPUT, 0u, len, buf);
}

static struct hid_device_ops kb_ops = {
	.iface_ready = kb_iface_ready,
	.get_report = kb_get_report,
	.set_report = kb_set_report,
	.set_idle = kb_set_idle,
	.get_idle = kb_get_idle,
	.set_protocol = kb_set_protocol,
	.output_report = kb_output_report,
};

int hid_keyboard_register(void)
{
	const struct device *hid_dev = DEVICE_DT_GET(DT_NODELABEL(hid_keyboard));
	int result = 0;

	if (!device_is_ready(hid_dev)) {
		return -EIO;
	}

	result = hid_device_register(hid_dev, hid_report_desc, sizeof(hid_report_desc), &kb_ops);
	if (result != 0) {
		return result;
	}

	return 0;
}
