/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENT_TEST_COMMON_H_
#define SENT_TEST_COMMON_H_

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sent/sent.h>
#include <zephyr/kernel.h>

#define SENT_TEST_CHANNEL	0U
#define SENT_TEST_MAX_FRAMES 100U

struct sent_test_rx_context {
	struct sent_frame frames[SENT_TEST_MAX_FRAMES];

	struct k_sem done;

	volatile uint32_t callback_count;
	volatile uint32_t frame_total;

	/*
	 * These objects must remain valid after sent_test_start_fast()
	 * returns. Keep them in the persistent receive context instead of
	 * declaring them as local stack variables.
	 */
	struct sent_rx_callback_config fast_config;
	struct sent_rx_callback_configs callback_configs;
};

int sent_test_rx_context_init(
	struct sent_test_rx_context *ctx);

void sent_test_rx_context_reset(
	struct sent_test_rx_context *ctx);

void sent_test_fast_cb(
	const struct device *dev,
	uint8_t channel,
	uint32_t num_frame,
	void *user_data);

int sent_test_start_fast(
	const struct device *dev,
	struct sent_test_rx_context *ctx,
	uint32_t batch_size);

int sent_test_stop(
	const struct device *dev);

#endif /* SENT_TEST_COMMON_H_ */
