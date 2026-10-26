/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>

#include "sent_taud_emulator.h"
#include "sent_test_common.h"
#include "sent_test_vectors.h"

#define SENT0_NODE DT_ALIAS(sent0)

#if !DT_NODE_EXISTS(SENT0_NODE)
#error "sent0 Devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS_OKAY(SENT0_NODE)
#error "sent0 Devicetree node is not enabled"
#endif

#define TEST_RX_TIMEOUT K_MSEC(250U)

static const struct device *const sent0 = DEVICE_DT_GET(SENT0_NODE);
static struct sent_test_rx_context rx0;
static bool sent0_started;

#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL)

/*
 * The multichannel overlay enables all required physical RSENT instances.
 * Do not use aliases here because the base overlay may map alias sent0 to a
 * different physical node for the normal single-channel tests.
 */
#define SENT_PHYS0_NODE  DT_NODELABEL(sent0)
#define SENT_PHYS1_NODE  DT_NODELABEL(sent1)
#define SENT_PHYS2_NODE  DT_NODELABEL(sent2)
#define SENT_PHYS3_NODE  DT_NODELABEL(sent3)
#define SENT_PHYS4_NODE  DT_NODELABEL(sent4)
#define SENT_PHYS13_NODE DT_NODELABEL(sent13)

#if DT_NODE_EXISTS(SENT_PHYS13_NODE) && DT_NODE_HAS_STATUS_OKAY(SENT_PHYS13_NODE)
#define MULTI_HAS_SENT13 1
#else
#define MULTI_HAS_SENT13 0
#endif

BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(SENT_PHYS0_NODE),
			 "multichannel requires physical SENT0");
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(SENT_PHYS1_NODE),
			 "multichannel requires physical SENT1");
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(SENT_PHYS2_NODE),
			 "multichannel requires physical SENT2");
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(SENT_PHYS3_NODE),
			 "multichannel requires physical SENT3");
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(SENT_PHYS4_NODE),
			 "multichannel requires physical SENT4");

#if MULTI_HAS_SENT13
BUILD_ASSERT(DT_NODE_HAS_STATUS_OKAY(SENT_PHYS13_NODE),
			 "multichannel SENT13 node must be enabled");
#define MULTI_SENT_COUNT 6U
#else
#define MULTI_SENT_COUNT 5U
#endif

static const struct device *const multi_sent_devs[MULTI_SENT_COUNT] = {
	DEVICE_DT_GET(SENT_PHYS0_NODE),
	DEVICE_DT_GET(SENT_PHYS1_NODE),
	DEVICE_DT_GET(SENT_PHYS2_NODE),
	DEVICE_DT_GET(SENT_PHYS3_NODE),
	DEVICE_DT_GET(SENT_PHYS4_NODE),
#if MULTI_HAS_SENT13
	DEVICE_DT_GET(SENT_PHYS13_NODE),
#endif
};

static struct sent_test_rx_context multi_rx[MULTI_SENT_COUNT];
static bool multi_started[MULTI_SENT_COUNT];

static uint32_t multi_sent_hw_channel(size_t index)
{
	static const uint32_t hw_channels[] = {
		0U, 1U, 2U, 3U, 4U,
#if MULTI_HAS_SENT13
		13U,
#endif
	};

	return hw_channels[index];
}

#endif /* CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL */

static void send_raw(const uint32_t *periods, size_t period_count)
{
	int ret = sent_taud_emulator_send_raw(periods, period_count);

	zassert_equal(ret, 0, "TAUD raw transmission failed: %d", ret);
}

static void start_fast(uint32_t frame_count)
{
	int ret = sent_test_start_fast(sent0, &rx0, frame_count);

	zassert_equal(ret, 0, "Failed to start Fast Channel reception: %d", ret);
	sent0_started = true;
}

static void wait_fast(void)
{
	int ret = k_sem_take(&rx0.done, TEST_RX_TIMEOUT);

	zassert_equal(ret, 0, "Timed out waiting for Fast Channel callback");
}

static void assert_zero_data_frame(const struct sent_frame *frame)
{
	zassert_equal(frame->type, SENT_FAST_FRAME);
	zassert_equal(frame->fast.data_nibbles[0], 0U);
	zassert_equal(frame->fast.data_nibbles[1], 0U);
	zassert_equal(frame->fast.data_nibbles[2], 0U);
	zassert_equal(frame->fast.data_nibbles[3], 0U);
	zassert_equal(frame->fast.data_nibbles[4], 0U);
	zassert_equal(frame->fast.data_nibbles[5], 0U);
}

static void assert_crc_payload_frame(const struct sent_frame *frame)
{
	zassert_equal(frame->type, SENT_FAST_FRAME);
	zassert_equal(frame->fast.data_nibbles[0], 0x1U);
	zassert_equal(frame->fast.data_nibbles[1], 0x2U);
	zassert_equal(frame->fast.data_nibbles[2], 0x3U);
	zassert_equal(frame->fast.data_nibbles[3], 0x4U);
	zassert_equal(frame->fast.data_nibbles[4], 0x5U);
	zassert_equal(frame->fast.data_nibbles[5], 0x6U);
}

static void *suite_setup(void)
{
	int ret;

	zassert_true(device_is_ready(sent0), "sent0 is not ready");

	ret = sent_test_rx_context_init(&rx0);
	zassert_equal(ret, 0, "Failed to initialize RX context: %d", ret);

#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL)
	for (size_t i = 0U; i < MULTI_SENT_COUNT; i++) {
		zassert_true(device_is_ready(multi_sent_devs[i]),
					 "Physical SENT%u is not ready", multi_sent_hw_channel(i));

		ret = sent_test_rx_context_init(&multi_rx[i]);
		zassert_equal(ret, 0,
					  "Failed to initialize physical SENT%u RX context: %d",
					  multi_sent_hw_channel(i), ret);
	}
#endif

	ret = sent_taud_emulator_open();
	zassert_equal(ret, 0, "Failed to open TAUD emulator: %d", ret);

	return NULL;
}

static void before(void *fixture)
{
	ARG_UNUSED(fixture);

	sent0_started = false;
	sent_test_rx_context_reset(&rx0);

#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL)
	for (size_t i = 0U; i < MULTI_SENT_COUNT; i++) {
		multi_started[i] = false;
		sent_test_rx_context_reset(&multi_rx[i]);
	}
#endif
}

static void after(void *fixture)
{
	int ret;

	ARG_UNUSED(fixture);

	if (sent0_started) {
		ret = sent_test_stop(sent0);
		zassert_equal(ret, 0, "Failed to stop SENT receiver: %d", ret);
		sent0_started = false;
	}

#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL)
	for (size_t i = 0U; i < MULTI_SENT_COUNT; i++) {
		if (!multi_started[i]) {
			continue;
		}

		ret = sent_test_stop(multi_sent_devs[i]);
		zassert_equal(ret, 0, "Failed to stop physical SENT%u: %d",
					  multi_sent_hw_channel(i), ret);
		multi_started[i] = false;
	}
#endif
}

static void suite_teardown(void *fixture)
{
	int ret;

	ARG_UNUSED(fixture);

	ret = sent_taud_emulator_close();
	zassert_equal(ret, 0, "Failed to close TAUD emulator: %d", ret);
}

ZTEST(sent_rh850_taud_raw, test_reference_vector_19_frames)
{
#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL) || \
	(CONFIG_SENT_TAUD_TEST_PAUSE_MODE != 0) || \
	(CONFIG_SENT_TAUD_TEST_CRC_MODE != 0)
	ztest_test_skip();
#else
	start_fast(FSP_REFERENCE_FRAME_COUNT);
	send_raw(fsp_enhanced_serial_format_0_pulses,
			 ARRAY_SIZE(fsp_enhanced_serial_format_0_pulses));
	wait_fast();

	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, FSP_REFERENCE_FRAME_COUNT);
	assert_zero_data_frame(&rx0.frames[0]);
	zassert_equal(rx0.frames[2].fast.data_nibbles[5], 0x1U);
	zassert_equal(rx0.frames[18].fast.data_nibbles[4], 0x1U);
	zassert_equal(rx0.frames[18].fast.data_nibbles[5], 0x1U);
#endif
}

ZTEST(sent_rh850_taud_raw, test_known_good_single_frame)
{
#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL) || \
	(CONFIG_SENT_TAUD_TEST_PAUSE_MODE != 0) || \
	(CONFIG_SENT_TAUD_TEST_CRC_MODE != 0)
	ztest_test_skip();
#else
	start_fast(1U);
	send_raw(known_good_single_frame, ARRAY_SIZE(known_good_single_frame));
	wait_fast();

	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 1U);
	assert_zero_data_frame(&rx0.frames[0]);
#endif
}

ZTEST(sent_rh850_taud_raw, test_calibration_error_recovery)
{
#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL) || \
	(CONFIG_SENT_TAUD_TEST_PAUSE_MODE != 0) || \
	(CONFIG_SENT_TAUD_TEST_CRC_MODE != 0)
	ztest_test_skip();
#else
	start_fast(1U);
	send_raw(bad_calibration_single_frame,
			 ARRAY_SIZE(bad_calibration_single_frame));
	k_msleep(20U);
	zassert_equal(rx0.callback_count, 0U,
				  "Invalid calibration unexpectedly produced a Fast callback");

	send_raw(known_good_single_frame, ARRAY_SIZE(known_good_single_frame));
	wait_fast();
	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 1U);
	assert_zero_data_frame(&rx0.frames[0]);
#endif
}

ZTEST(sent_rh850_taud_raw, test_crc_valid_raw)
{
#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL) || \
	(CONFIG_SENT_TAUD_TEST_PAUSE_MODE != 0)
	ztest_test_skip();
#elif CONFIG_SENT_TAUD_TEST_CRC_MODE == 1
	start_fast(1U);
	send_raw(recommended_crc_single_frame,
			 ARRAY_SIZE(recommended_crc_single_frame));
	wait_fast();
	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 1U);
	assert_crc_payload_frame(&rx0.frames[0]);
#elif CONFIG_SENT_TAUD_TEST_CRC_MODE == 2
	start_fast(1U);
	send_raw(legacy_crc_single_frame, ARRAY_SIZE(legacy_crc_single_frame));
	wait_fast();
	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 1U);
	assert_crc_payload_frame(&rx0.frames[0]);
#else
	ztest_test_skip();
#endif
}

ZTEST(sent_rh850_taud_raw, test_crc_error_recovery_raw)
{
#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL) || \
	(CONFIG_SENT_TAUD_TEST_PAUSE_MODE != 0) || \
	(CONFIG_SENT_TAUD_TEST_CRC_MODE == 0)
	ztest_test_skip();
#elif CONFIG_SENT_TAUD_TEST_CRC_MODE == 1
	start_fast(1U);
	send_raw(bad_recommended_crc_single_frame,
			 ARRAY_SIZE(bad_recommended_crc_single_frame));
	k_msleep(20U);
	zassert_equal(rx0.callback_count, 0U);

	send_raw(recommended_crc_single_frame,
			 ARRAY_SIZE(recommended_crc_single_frame));
	wait_fast();
	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 1U);
	assert_crc_payload_frame(&rx0.frames[0]);
#else
	start_fast(1U);
	send_raw(bad_legacy_crc_single_frame,
			 ARRAY_SIZE(bad_legacy_crc_single_frame));
	k_msleep(20U);
	zassert_equal(rx0.callback_count, 0U);

	send_raw(legacy_crc_single_frame, ARRAY_SIZE(legacy_crc_single_frame));
	wait_fast();
	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 1U);
	assert_crc_payload_frame(&rx0.frames[0]);
#endif
}

ZTEST(sent_rh850_taud_raw, test_pause_pulse_raw)
{
#if IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL) || \
	(CONFIG_SENT_TAUD_TEST_PAUSE_MODE == 0)
	ztest_test_skip();
#elif CONFIG_SENT_TAUD_TEST_PAUSE_MODE == 1
	start_fast(4U);
	send_raw(variable_pause_frames, ARRAY_SIZE(variable_pause_frames));
	wait_fast();

	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 4U);
	assert_zero_data_frame(&rx0.frames[0]);
	assert_zero_data_frame(&rx0.frames[1]);
	assert_zero_data_frame(&rx0.frames[2]);
	assert_zero_data_frame(&rx0.frames[3]);
#else
	start_fast(4U);
	send_raw(fixed_pause_frames, ARRAY_SIZE(fixed_pause_frames));
	wait_fast();

	zassert_equal(rx0.callback_count, 1U);
	zassert_equal(rx0.frame_total, 4U);
	assert_zero_data_frame(&rx0.frames[0]);
	assert_zero_data_frame(&rx0.frames[1]);
	assert_zero_data_frame(&rx0.frames[2]);
	assert_zero_data_frame(&rx0.frames[3]);
#endif
}

/*
 * Keep this ZTEST visible in every configuration so Twister receives an
 * explicit SKIP instead of retaining a None status in non-multichannel runs.
 */
ZTEST(sent_rh850_taud_raw, test_multichannel_same_stream_all_receivers)
{
#if !IS_ENABLED(CONFIG_SENT_TAUD_TEST_MULTI_CHANNEL)
	ztest_test_skip();
#else
	int ret;

	/*
	 * Arm every physical receiver first. sent_test_start_fast() registers
	 * sent_test_fast_cb() with &multi_rx[i] as user_data and starts listening.
	 */
	for (size_t i = 0U; i < MULTI_SENT_COUNT; i++) {
		ret = sent_test_start_fast(multi_sent_devs[i], &multi_rx[i],
								   FSP_REFERENCE_FRAME_COUNT);
		zassert_equal(ret, 0,
					  "Failed to start physical SENT%u: %d",
					  multi_sent_hw_channel(i), ret);
		multi_started[i] = true;
	}

	/* One TAUD transmission is observed by every wired physical receiver. */
	send_raw(fsp_enhanced_serial_format_0_pulses,
			 ARRAY_SIZE(fsp_enhanced_serial_format_0_pulses));

	/* The callback gives multi_rx[i].done through its user_data context. */
	for (size_t i = 0U; i < MULTI_SENT_COUNT; i++) {
		ret = k_sem_take(&multi_rx[i].done, TEST_RX_TIMEOUT);

		if (ret != 0) {
			printk("SENT timeout: index=%u physical=%u ret=%d "
				   "callback_count=%u frame_total=%u\n",
				   (unsigned int)i,
				   multi_sent_hw_channel(i),
				   ret,
				   multi_rx[i].callback_count,
				   multi_rx[i].frame_total);
		}

		zassert_equal(ret, 0,
					  "Timed out waiting for physical SENT%u callback: ret=%d",
					  multi_sent_hw_channel(i), ret);
	}

	for (size_t i = 0U; i < MULTI_SENT_COUNT; i++) {
		zassert_equal(multi_rx[i].callback_count, 1U,
				"Unexpected physical SENT%u callback count: %u",
				multi_sent_hw_channel(i),
				multi_rx[i].callback_count);

		zassert_equal(multi_rx[i].frame_total,
				FSP_REFERENCE_FRAME_COUNT,
				"Unexpected physical SENT%u frame total: %u",
				multi_sent_hw_channel(i),
				multi_rx[i].frame_total);

		assert_zero_data_frame(&multi_rx[i].frames[0]);

		TC_PRINT("SENT%u:\n", multi_sent_hw_channel(i));

		for (size_t frame = 0U; frame < FSP_REFERENCE_FRAME_COUNT; frame++) {
			const struct sent_frame *f = &multi_rx[i].frames[frame];

			TC_PRINT(
				"frame %02u: %X %X %X %X %X %X "
				"(exp4=%X exp5=%X)\n",
				frame,
				f->fast.data_nibbles[0],
				f->fast.data_nibbles[1],
				f->fast.data_nibbles[2],
				f->fast.data_nibbles[3],
				f->fast.data_nibbles[4],
				f->fast.data_nibbles[5],
				expected_nibble4[frame],
				expected_final_nibble[frame]);

			zassert_equal(
				f->fast.data_nibbles[4],
				expected_nibble4[frame],
				"Physical SENT%u frame %u nibble4 mismatch",
				multi_sent_hw_channel(i),
				frame);

			zassert_equal(
				f->fast.data_nibbles[5],
				expected_final_nibble[frame],
				"Physical SENT%u frame %u nibble5 mismatch",
				multi_sent_hw_channel(i),
				frame);
		}
	}
#endif
}

ZTEST_SUITE(sent_rh850_taud_raw,
			NULL,
			suite_setup,
			before,
			after,
			suite_teardown);
