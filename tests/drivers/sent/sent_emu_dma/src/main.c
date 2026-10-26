/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
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

#define SENT_FAST_FRAME_COUNT 19U
#define SENT_SERIAL_MESSAGE_COUNT 1U
#define SENT_FAST_FRAME_PERIOD_COUNT 9U
#define SENT_STREAM_TERMINATION_PERIOD_COUNT 2U

#define SENT_EXPECTED_SERIAL_ID 0xBCU
#define SENT_EXPECTED_SERIAL_DATA 0xDEFU

#define SENT_RX_TIMEOUT K_MSEC(100U)

#if !DT_NODE_EXISTS(SENT_NODE)
#error "sent0 Devicetree alias is not defined"
#endif

#if !DT_DMAS_HAS_NAME(DT_CHILD(SENT_NODE, channel_0), rx)
#error "The SENT test instance must define an RX DMA specifier"
#endif

static const struct device *const sent_dev = DEVICE_DT_GET(SENT_NODE);

static struct sent_frame fast_frames[SENT_FAST_FRAME_COUNT];
static struct sent_frame serial_frames[SENT_SERIAL_MESSAGE_COUNT];

K_SEM_DEFINE(fast_rx_done, 0, 1);
K_SEM_DEFINE(serial_rx_done, 0, 1);

static volatile uint32_t fast_callback_count;
static volatile uint32_t fast_frame_total;
static volatile uint32_t serial_callback_count;
static volatile uint32_t serial_message_total;

static const uint32_t fsp_sent_pulses[] = {
	/* Dummy frame. */
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

	/* Stream termination periods. */
	56U,
	56U,
};

BUILD_ASSERT(
	ARRAY_SIZE(fsp_sent_pulses) ==
		((SENT_FAST_FRAME_COUNT * SENT_FAST_FRAME_PERIOD_COUNT) +
		 SENT_STREAM_TERMINATION_PERIOD_COUNT),
	"Unexpected FSP pulse-vector size");

static void rx_fast_frame_cb(const struct device *dev, uint8_t channel_id,
				 uint32_t num_frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if ((channel_id != SENT_CHANNEL) || (num_frame == 0U)) {
		return;
	}

	fast_callback_count++;
	fast_frame_total += num_frame;
	k_sem_give(&fast_rx_done);
}

static void rx_serial_frame_cb(const struct device *dev, uint8_t channel_id,
				   uint32_t num_frame, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	if ((channel_id != SENT_CHANNEL) || (num_frame == 0U)) {
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

static int sent_test_start_and_send(struct sent_rx_callback_configs callbacks)
{
	int ret;

	ret = sent_register_callback(sent_dev, SENT_CHANNEL, callbacks);
	if (ret != 0) {
		return ret;
	}

	ret = sent_start_listening(sent_dev, SENT_CHANNEL);
	if (ret != 0) {
		return ret;
	}

	k_msleep(10U);

	return sent_taud_emulator_send_raw(fsp_sent_pulses,
					   ARRAY_SIZE(fsp_sent_pulses));
}

static void *sent_test_setup(void)
{
	int ret;

	zassert_true(device_is_ready(sent_dev), "SENT device is not ready");

	ret = sent_taud_emulator_open();
	zassert_equal(ret, 0, "Failed to open TAUD SENT emulator: %d", ret);

	return NULL;
}

static void sent_test_before(void *fixture)
{
	ARG_UNUSED(fixture);
	sent_test_runtime_reset();
}

static void sent_test_after(void *fixture)
{
	int ret;

	ARG_UNUSED(fixture);

	ret = sent_stop_listening(sent_dev, SENT_CHANNEL);
	zassert_true((ret == 0) || (ret == -EALREADY),
			 "Failed to stop SENT channel: %d", ret);
}

static void sent_test_teardown(void *fixture)
{
	int ret;

	ARG_UNUSED(fixture);

	ret = sent_taud_emulator_close();
	zassert_equal(ret, 0, "Failed to close TAUD SENT emulator: %d", ret);
}

ZTEST(sent_renesas_rh850_dma, test_receive_fast_channel_dma)
{
	struct sent_rx_callback_configs callbacks = {
		.fast = &fast_cb_cfg,
		.serial = NULL,
	};
	int ret;

	ret = sent_test_start_and_send(callbacks);
	zassert_equal(ret, 0, "Failed to start/send Fast DMA test: %d", ret);

	ret = k_sem_take(&fast_rx_done, SENT_RX_TIMEOUT);
	zassert_equal(ret, 0, "Timed out waiting for Fast DMA callback");

	zassert_equal(fast_callback_count, 1U,
			  "Unexpected Fast callback count: %u", fast_callback_count);
	zassert_equal(fast_frame_total, SENT_FAST_FRAME_COUNT,
			  "Unexpected Fast frame count: %u", fast_frame_total);
	zassert_equal(serial_callback_count, 0U,
			  "Serial callback unexpectedly ran: %u", serial_callback_count);

	zassert_equal(fast_frames[0].fast.data_nibbles[0], 0x0U,
			  "Unexpected dummy-frame data nibble");
	zassert_equal(fast_frames[1].fast.data_nibbles[5], 0x0U,
			  "Unexpected first Enhanced frame data nibble");
	zassert_equal(fast_frames[2].fast.data_nibbles[5], 0x1U,
			  "Unexpected second Enhanced frame data nibble");
	zassert_equal(fast_frames[18].fast.data_nibbles[4], 0x1U,
			  "Unexpected final Enhanced frame data nibble 4");
	zassert_equal(fast_frames[18].fast.data_nibbles[5], 0x1U,
			  "Unexpected final Enhanced frame data nibble 5");
}

ZTEST(sent_renesas_rh850_dma, test_receive_serial_channel_cpu_driven)
{
	struct sent_rx_callback_configs callbacks = {
		.fast = NULL,
		.serial = &serial_cb_cfg,
	};
	int ret;

	/*
	 * Only the serial callback is registered. The RSENT driver must keep the
	 * fast receive path moving internally and read the completed serial
	 * message from SRXD using the CPU-driven path.
	 */
	ret = sent_test_start_and_send(callbacks);
	zassert_equal(ret, 0, "Failed to start/send Serial CPU test: %d", ret);

	ret = k_sem_take(&serial_rx_done, SENT_RX_TIMEOUT);
	zassert_equal(ret, 0, "Timed out waiting for Serial CPU callback");

	zassert_equal(serial_callback_count, 1U,
			  "Unexpected Serial callback count: %u", serial_callback_count);
	zassert_equal(serial_message_total, SENT_SERIAL_MESSAGE_COUNT,
			  "Unexpected Serial message count: %u", serial_message_total);
	zassert_equal(fast_callback_count, 0U,
			  "Fast callback unexpectedly ran: %u", fast_callback_count);

	zassert_equal(serial_frames[0].type,
			  SENT_ENHANCED_SERIAL_FRAME_8_BIT_ID,
			  "Unexpected serial frame type: %u",
			  (uint32_t)serial_frames[0].type);
	zassert_equal(serial_frames[0].serial.id,
			  SENT_EXPECTED_SERIAL_ID,
			  "Unexpected serial message ID: 0x%X",
			  serial_frames[0].serial.id);
	zassert_equal(serial_frames[0].serial.data,
			  SENT_EXPECTED_SERIAL_DATA,
			  "Unexpected serial message data: 0x%X",
			  serial_frames[0].serial.data);
}

ZTEST_SUITE(sent_renesas_rh850_dma, NULL, sent_test_setup,
		sent_test_before, sent_test_after, sent_test_teardown);
