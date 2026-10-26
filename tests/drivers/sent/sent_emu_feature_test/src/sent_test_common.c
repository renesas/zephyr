/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include "sent_test_common.h"

#include <errno.h>
#include <string.h>

void sent_test_rx_context_reset(struct sent_test_rx_context *ctx)
{
	memset(ctx->frames, 0, sizeof(ctx->frames));
	ctx->callback_count = 0U;
	ctx->frame_total = 0U;
	k_sem_reset(&ctx->done);
}

void sent_test_fast_cb(const struct device *dev, uint8_t channel,
					   uint32_t num_frame, void *user_data)
{
	struct sent_test_rx_context *ctx = user_data;

	ARG_UNUSED(dev);

	if ((channel != SENT_TEST_CHANNEL) || (num_frame == 0U) || (ctx == NULL)) {
		return;
	}

	ctx->callback_count++;
	ctx->frame_total += num_frame;

	/*
	 * For multichannel operation, user_data points to the matching
	 * multi_rx[i] context, so this releases that receiver's own waiter.
	 */
	k_sem_give(&ctx->done);
}

int sent_test_rx_context_init(struct sent_test_rx_context *ctx)
{
	int ret;

	memset(ctx, 0, sizeof(*ctx));

	ret = k_sem_init(&ctx->done, 0U, 1U);
	if (ret != 0) {
		return ret;
	}

	ctx->fast_config.callback = sent_test_fast_cb;
	ctx->fast_config.frame = ctx->frames;
	ctx->fast_config.max_num_frame = ARRAY_SIZE(ctx->frames);
	ctx->fast_config.user_data = ctx;
	ctx->callback_configs.fast = &ctx->fast_config;
	ctx->callback_configs.serial = NULL;

	return 0;
}

int sent_test_start_fast(const struct device *dev,
						 struct sent_test_rx_context *ctx,
						 uint32_t batch_size)
{
	int ret;

	if ((batch_size == 0U) || (batch_size > ARRAY_SIZE(ctx->frames))) {
		return -EINVAL;
	}

	ctx->fast_config.max_num_frame = batch_size;

	ret = sent_register_callback(dev, SENT_TEST_CHANNEL,
								 ctx->callback_configs);
	if (ret != 0) {
		return ret;
	}

	ret = sent_start_listening(dev, SENT_TEST_CHANNEL);
	if (ret == 0) {
		k_msleep(10U);
	}

	return ret;
}

int sent_test_stop(const struct device *dev)
{
	int ret = sent_stop_listening(dev, SENT_TEST_CHANNEL);

	return (ret == -EALREADY) ? 0 : ret;
}
