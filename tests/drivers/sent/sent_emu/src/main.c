/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sent/sent.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "sent_taud_emulator.h"

#define SENT_NODE DT_ALIAS(sent0)
#define SENT_CHANNEL 0U

/*
 * The FSP TG4 Enhanced Serial Format 0 vector contains:
 *
 *     1 dummy fast frame
 *     18 Enhanced Serial fast frames
 */
#define SENT_FAST_FRAME_COUNT 19U
#define SENT_SERIAL_MESSAGE_COUNT 1U

/*
 * One Fast Channel frame in this test contains:
 *
 *     calibration pulse
 *     Status/Communication nibble
 *     six data nibbles
 *     Fast CRC nibble
 */
#define SENT_FAST_FRAME_PERIOD_COUNT 9U

/*
 * The FSP vector contains two trailing calibration periods.
 */
#define SENT_STREAM_TERMINATION_PERIOD_COUNT 2U

#define SENT_EXPECTED_SERIAL_ID   0xBCU
#define SENT_EXPECTED_SERIAL_DATA 0xDEFU

/*
 * Callback wait timeout.
 *
 * TAUD transmission itself is synchronous, but the RSENT callback may run
 * immediately before or shortly after sent_taud_emulator_send_raw()
 * returns.
 */
#define SENT_RX_TIMEOUT K_MSEC(100U)

#if !DT_NODE_EXISTS(SENT_NODE)
#error "sent0 Devicetree alias is not defined"
#endif

static const struct device *const sent_dev =
	DEVICE_DT_GET(SENT_NODE);

static struct sent_frame
	fast_frames[SENT_FAST_FRAME_COUNT];

static struct sent_frame
	serial_frames[SENT_SERIAL_MESSAGE_COUNT];

/*
 * The callbacks execute from the FSP/RSENT interrupt path.
 *
 * Semaphores provide interrupt-safe notification to the ztest thread.
 */
K_SEM_DEFINE(fast_rx_done, 0, 1);
K_SEM_DEFINE(serial_rx_done, 0, 1);

static volatile uint32_t fast_callback_count;
static volatile uint32_t fast_frame_total;
static volatile uint32_t serial_callback_count;
static volatile uint32_t serial_message_total;

/*
 * Exact pulse vector copied from:
 *
 *     g_sent_tg4_tick_count_arr_enhanced_serial_format_0
 *
 * SENT_EMU_END_NIBBLE_MARK is omitted because the Zephyr TAUD emulator
 * receives an explicit pulse count.
 *
 * The two trailing 56-tick periods are preserved.
 */
static const uint32_t fsp_enhanced_serial_format_0_pulses[] = {
	/*
	 * Dummy frame.
	 *
	 * Provides a zero immediately before the Enhanced Serial bit-3
	 * synchronization sequence 1111110.
	 */
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U,

	/* Enhanced Serial frames 1 through 6. */
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 12U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 13U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 14U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 15U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 16U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 17U, 17U,

	/* Enhanced Serial frames 7 through 10. */
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 18U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 19U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 20U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 21U, 17U,

	/* Enhanced Serial frames 11 through 12. */
	56U, 27U, 12U, 12U, 12U, 12U, 12U, 22U, 17U,
	56U, 27U, 12U, 12U, 12U, 12U, 12U, 23U, 17U,

	/* Enhanced Serial frames 13 through 16. */
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 24U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 25U, 17U,
	56U, 27U, 12U, 12U, 12U, 12U, 12U, 26U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 27U, 17U,

	/* Enhanced Serial frames 17 through 18. */
	56U, 19U, 12U, 12U, 12U, 12U, 13U, 12U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 13U, 13U, 17U,

	/*
	 * Stream termination periods copied from the FSP vector.
	 */
	56U,
	56U,
};

BUILD_ASSERT(
	ARRAY_SIZE(fsp_enhanced_serial_format_0_pulses) ==
		((SENT_FAST_FRAME_COUNT *
		  SENT_FAST_FRAME_PERIOD_COUNT) +
		 SENT_STREAM_TERMINATION_PERIOD_COUNT),
	"Unexpected FSP Enhanced Serial pulse-vector size");

static void rx_fast_frame_cb(
	const struct device *dev,
	uint8_t channel_id,
	uint32_t num_frame,
	void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	/*
	 * Do not perform zassert or logging here.
	 *
	 * The callback may execute in interrupt context. Store the result and
	 * wake the test thread instead.
	 */
	if ((channel_id != SENT_CHANNEL) ||
	    (num_frame == 0U)) {
		return;
	}

	fast_callback_count++;
	fast_frame_total += num_frame;

	k_sem_give(&fast_rx_done);
}

static void rx_serial_frame_cb(
	const struct device *dev,
	uint8_t channel_id,
	uint32_t num_frame,
	void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if ((channel_id != SENT_CHANNEL) ||
	    (num_frame == 0U)) {
		return;
	}

	serial_callback_count++;
	serial_message_total += num_frame;

	k_sem_give(&serial_rx_done);
}

static struct sent_rx_callback_config fast_cb_cfg = {
	.callback = rx_fast_frame_cb,
	.frame = fast_frames,
	.max_num_frame = ARRAY_SIZE(fast_frames),
	.user_data = NULL,
};

static struct sent_rx_callback_config serial_cb_cfg = {
	.callback = rx_serial_frame_cb,
	.frame = serial_frames,
	.max_num_frame = ARRAY_SIZE(serial_frames),
	.user_data = NULL,
};

static struct sent_rx_callback_configs callback_configs = {
	.fast = &fast_cb_cfg,
	.serial = &serial_cb_cfg,
};

static void sent_test_runtime_reset(void)
{
	memset(fast_frames, 0, sizeof(fast_frames));
	memset(serial_frames, 0, sizeof(serial_frames));

	fast_callback_count = 0U;
	fast_frame_total = 0U;
	serial_callback_count = 0U;
	serial_message_total = 0U;

	k_sem_reset(&fast_rx_done);
	k_sem_reset(&serial_rx_done);
}

static void *sent_test_setup(void)
{
	int ret;

	zassert_true(
		device_is_ready(sent_dev),
		"SENT device is not ready");

	ret = sent_taud_emulator_open();

	zassert_equal(
		ret,
		0,
		"Failed to open TAUD SENT emulator: %d",
		ret);

	return NULL;
}

static void sent_test_before(void *fixture)
{
	ARG_UNUSED(fixture);

	sent_test_runtime_reset();
}

static void sent_test_after(void *fixture)
{
	ARG_UNUSED(fixture);

	/*
	 * Stop reception after every test so each test begins from a known
	 * RSENT state.
	 *
	 * Ignore -EALREADY because a test may fail before the channel starts.
	 */
	int ret = sent_stop_listening(
		sent_dev,
		SENT_CHANNEL);

	zassert_true(
		(ret == 0) || (ret == -EALREADY),
		"Failed to stop SENT channel: %d",
		ret);
}

static void sent_test_teardown(void *fixture)
{
	int ret;

	ARG_UNUSED(fixture);

	ret = sent_taud_emulator_close();

	zassert_equal(
		ret,
		0,
		"Failed to close TAUD SENT emulator: %d",
		ret);
}

ZTEST(sent_renesas_rh850, test_receive_serial_fast_transmission)
{
	int ret;

	ret = sent_register_callback(
		sent_dev,
		SENT_CHANNEL,
		callback_configs);

	zassert_equal(
		ret,
		0,
		"Failed to register SENT callbacks: %d",
		ret);

	ret = sent_start_listening(
		sent_dev,
		SENT_CHANNEL);

	zassert_equal(
		ret,
		0,
		"Failed to start SENT channel %u: %d",
		SENT_CHANNEL,
		ret);

	/*
	 * Match the FSP test flow.
	 *
	 * FSP polls StatusGet() until ACTIVE. The generic Zephyr SENT API
	 * currently has no status-get operation, so allow the hardware a
	 * short interval to enter ACTIVE state.
	 */
	k_msleep(10U);

	ret = sent_taud_emulator_send_raw(
		fsp_enhanced_serial_format_0_pulses,
		ARRAY_SIZE(
			fsp_enhanced_serial_format_0_pulses));

	zassert_equal(
		ret,
		0,
		"TAUD raw pulse transmission failed: %d",
		ret);

	/*
	 * Wait independently for both receive paths.
	 *
	 * The callbacks may already have given these semaphores before
	 * send_raw() returns. k_sem_take() still succeeds in that case.
	 */
	ret = k_sem_take(
		&fast_rx_done,
		SENT_RX_TIMEOUT);

	zassert_equal(
		ret,
		0,
		"Timed out waiting for Fast Channel callback");

	ret = k_sem_take(
		&serial_rx_done,
		SENT_RX_TIMEOUT);

	zassert_equal(
		ret,
		0,
		"Timed out waiting for Enhanced Serial callback");

	zassert_equal(
		fast_callback_count,
		1U,
		"Unexpected Fast callback count: %u",
		fast_callback_count);

	zassert_equal(
		fast_frame_total,
		SENT_FAST_FRAME_COUNT,
		"Unexpected Fast frame count: %u",
		fast_frame_total);

	zassert_equal(
		serial_callback_count,
		1U,
		"Unexpected Serial callback count: %u",
		serial_callback_count);

	zassert_equal(
		serial_message_total,
		SENT_SERIAL_MESSAGE_COUNT,
		"Unexpected Serial message count: %u",
		serial_message_total);

	zassert_equal(
		serial_frames[0].type,
		SENT_ENHANCED_SERIAL_FRAME_8_BIT_ID,
		"Unexpected serial frame type: %u",
		(uint32_t)serial_frames[0].type);

	zassert_equal(
		serial_frames[0].serial.id,
		SENT_EXPECTED_SERIAL_ID,
		"Unexpected serial message ID: 0x%X",
		serial_frames[0].serial.id);

	zassert_equal(
		serial_frames[0].serial.data,
		SENT_EXPECTED_SERIAL_DATA,
		"Unexpected serial message data: 0x%X",
		serial_frames[0].serial.data);

	/*
	 * Verify selected Fast Channel payload values from the raw vector.
	 *
	 * FSP packs the first pulse data nibble as the least significant
	 * nibble. The Renesas Zephyr driver unpacks data in that order.
	 */
	zassert_equal(
		fast_frames[0].fast.data_nibbles[0],
		0x0U,
		"Unexpected dummy-frame data nibble");

	zassert_equal(
		fast_frames[1].fast.data_nibbles[5],
		0x0U,
		"Unexpected first Enhanced frame data nibble");

	zassert_equal(
		fast_frames[2].fast.data_nibbles[5],
		0x1U,
		"Unexpected second Enhanced frame data nibble");

	zassert_equal(
		fast_frames[18].fast.data_nibbles[4],
		0x1U,
		"Unexpected final Enhanced frame data nibble 4");

	zassert_equal(
		fast_frames[18].fast.data_nibbles[5],
		0x1U,
		"Unexpected final Enhanced frame data nibble 5");
}

ZTEST_SUITE(
	sent_renesas_rh850,
	NULL,
	sent_test_setup,
	sent_test_before,
	sent_test_after,
	sent_test_teardown);
