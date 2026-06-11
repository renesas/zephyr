/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(renesas_rh850_crc, CONFIG_CRC_LOG_LEVEL);

#include <errno.h>
#include <zephyr/drivers/crc.h>

#include <soc.h>
#include "r_crc.h"
#include "rp_crc.h"

#define DT_DRV_COMPAT     renesas_rh850_crc
#define DEFAULT_NUM_BYTES (4U)

struct crc_renesas_rh850_cfg {
	const crc_api_t *fsp_api;
};

struct crc_renesas_rh850_data {
	struct st_crc_instance_ctrl ctrl;
	struct st_crc_input_t input_data;
	struct st_crc_cfg crc_config;
	struct k_sem sem;
};

static void crc_lock(const struct device *dev)
{
	struct crc_renesas_rh850_data *data = dev->data;

	k_sem_take(&data->sem, K_FOREVER);
}

static void crc_unlock(const struct device *dev)
{
	struct crc_renesas_rh850_data *data = dev->data;

	k_sem_give(&data->sem);
}

static int crc_set_config(const struct device *dev, struct crc_ctx *ctx)
{
	fsp_err_t err;
	struct crc_renesas_rh850_data *data = dev->data;
	crc_cfg_t *crc_cfg = &data->crc_config;
	crc_extended_cfg_t *p_extend = (crc_extended_cfg_t *)crc_cfg->p_extend;
	crc_ctl_reg_t *p_ctl = (crc_ctl_reg_t *) p_extend->control_reg;

	crc_cfg->bit_order = CRC_BIT_ORDER_LMS_MSB;

	switch (ctx->type) {
	case CRC8: {
		if ((ctx->polynomial != CRC8_POLY) && (ctx->polynomial != CRC8_REFLECT_POLY)) {
			return -EINVAL;
		}
		p_extend->xor0 = 0x0U;
		p_ctl->poly_size = CRC_POLYNOMIAL_SIZE_8_BITS;
		break;
	}
	case CRC16: {
		if ((ctx->polynomial != CRC16_POLY) && (ctx->polynomial != CRC16_REFLECT_POLY)) {
			return -EINVAL;
		}
		p_extend->xor0 = 0x0U;
		p_ctl->poly_size = CRC_POLYNOMIAL_SIZE_16_BITS;
		break;
	}
	case CRC16_CCITT: {
		if (ctx->polynomial != CRC16_CCITT_POLY) {
			return -EINVAL;
		}
		p_extend->xor0 = 0x0U;
		p_ctl->poly_size = CRC_POLYNOMIAL_SIZE_16_BITS;
		break;
	}
	case CRC32_C: {
		if (ctx->polynomial != CRC32C_POLY) {
			return -EINVAL;
		}
		p_extend->xor0 = 0x0U;
		p_ctl->poly_size = CRC_POLYNOMIAL_SIZE_32_BITS;
		break;
	}
	case CRC32_IEEE: {
		if (ctx->polynomial != CRC32_IEEE_POLY) {
			return -EINVAL;
		}
		p_extend->xor0 = 0xFFFFFFFFU;
		p_ctl->poly_size = CRC_POLYNOMIAL_SIZE_32_BITS;
		break;
	}
	default:
		return -ENOTSUP;
	}

	p_extend->poly0 = ctx->polynomial;
	if ((ctx->polynomial == CRC16_REFLECT_POLY) || (ctx->polynomial == CRC8_REFLECT_POLY)) {
		p_ctl->output_mode = CRC_OUTPUT_MODE_NORMAL;
		p_ctl->input_mode = CRC_INPUT_MODE_NORMAL;
		crc_cfg->bit_order = CRC_BIT_ORDER_LMS_LSB;
	} else {
		p_ctl->output_mode =
			(crc_output_mode_t)((ctx->reversed & CRC_FLAG_REVERSE_OUTPUT) >> 1);
		p_ctl->input_mode =
			(crc_input_mode_t)(ctx->reversed & CRC_FLAG_REVERSE_INPUT);
	}
	p_extend->dout0 = ctx->seed;

	crc_cfg->p_extend = (void *)p_extend;

	err = RP_CRC_Reconfigure(&data->ctrl, crc_cfg);

	if (err != FSP_SUCCESS) {
		ctx->state = CRC_STATE_IDLE;
		return -EINVAL;
	}

	return 0;
}

static int crc_renesas_rh850_begin(const struct device *dev, struct crc_ctx *ctx)
{
	int ret;

	crc_lock(dev);

	ret = crc_set_config(dev, ctx);
	if (ret != 0) {
		crc_unlock(dev);
		return ret;
	}

	ctx->state = CRC_STATE_IN_PROGRESS;

	return 0;
}

static int crc_renesas_rh850_update(const struct device *dev, struct crc_ctx *ctx,
				      const void *buffer, size_t bufsize)
{
	struct crc_renesas_rh850_data *data = dev->data;
	const struct crc_renesas_rh850_cfg *cfg = dev->config;

	/* Ensure CRC calculation has been initialized by crc_begin() */
	if (ctx->state == CRC_STATE_IDLE) {
		return -EINVAL;
	}

	data->input_data.num_bytes = bufsize;
	data->input_data.p_input_buffer = (uint8_t *)buffer;

	if (FSP_SUCCESS !=
	    (cfg->fsp_api->calculate(&data->ctrl, &data->input_data, (uint64_t *) &ctx->result))) {
		ctx->state = CRC_STATE_IDLE;
		crc_unlock(dev);
		return -EINVAL;
	}

	return 0;
}

static int crc_renesas_rh850_finish(const struct device *dev, struct crc_ctx *ctx)
{
	struct crc_renesas_rh850_data *data = dev->data;
	const struct crc_renesas_rh850_cfg *cfg = dev->config;

	if (ctx->state == CRC_STATE_IDLE) {
		return -EINVAL;
	}

	if (FSP_SUCCESS != (cfg->fsp_api->close(&data->ctrl))) {
		return -EINVAL;
	}

	ctx->state = CRC_STATE_IDLE;

	crc_unlock(dev);

	return 0;
}

static int crc_rh850_init(const struct device *dev)
{
	const struct crc_renesas_rh850_cfg *cfg = dev->config;
	struct crc_renesas_rh850_data *data = dev->data;
	crc_cfg_t *crc_cfg = &data->crc_config;

	if (FSP_SUCCESS != (cfg->fsp_api->open(&data->ctrl, crc_cfg))) {
		return -EINVAL;
	}

	k_sem_init(&data->sem, 1, 1);

	return 0;
}

static DEVICE_API(crc, crc_renesas_rh850_driver_api) = {
	.begin = crc_renesas_rh850_begin,
	.update = crc_renesas_rh850_update,
	.finish = crc_renesas_rh850_finish,
};

#define CRC_RH850_INIT_CFG(idx)                                                                  \
	static const struct crc_renesas_rh850_cfg crc_renesas_rh850_cfg_##idx = {              \
		.fsp_api = &g_crc_on_crc,                                                          \
	};

#define CRC_RH850_INIT(idx)                                                                      \
	CRC_RH850_INIT_CFG(idx);                                                                 \
	static crc_ctl_reg_t g_crc##idx##_ctr_cfg = {                                              \
		.input_data_size = CRC_INPUT_DATA_SIZE_8_BITS,                                     \
		.input_mode = CRC_INPUT_MODE_REFLECT,                                              \
		.output_mode = CRC_OUTPUT_MODE_REFLECT,                                            \
		.poly_size = CRC_POLYNOMIAL_SIZE_32_BITS,                                          \
	};                                                                                         \
	static crc_extended_cfg_t g_crc##idx##_cfg_extend = {                                      \
		.control_reg = (crc_ctl_reg_t *)&g_crc##idx##_ctr_cfg,                             \
		.poly0 = 0x4C11DB7,                                                                \
		.poly1 = 0x0,                                                                      \
		.dout0 = 0xFFFFFFFF,                                                               \
		.dout1 = 0x0,                                                                      \
		.xor0 = 0xFFFFFFFF,                                                                \
		.xor1 = 0x0,                                                                       \
	};                                                                                         \
	static struct crc_renesas_rh850_data crc_renesas_rh850_data_##idx = {                  \
		.input_data =                                                                      \
			{                                                                          \
				.num_bytes = DEFAULT_NUM_BYTES,                                    \
				.p_input_buffer = NULL,                                            \
			},                                                                         \
		.crc_config =                                                                      \
			{                                                                          \
				.channel = DT_INST_PROP(idx, channel),                             \
				.bit_order = CRC_BIT_ORDER_LMS_LSB,                                \
				.p_extend = &g_crc##idx##_cfg_extend,                              \
			},                                                                         \
	};                                                                                         \
	static int crc_rh850_init_##idx(const struct device *dev)                                \
	{                                                                                          \
		return crc_rh850_init(dev);                                                      \
	}                                                                                          \
	DEVICE_DT_INST_DEFINE(idx, &crc_rh850_init_##idx, NULL, &crc_renesas_rh850_data_##idx, \
			      &crc_renesas_rh850_cfg_##idx, POST_KERNEL,                         \
			      CONFIG_CRC_DRIVER_INIT_PRIORITY, &crc_renesas_rh850_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CRC_RH850_INIT)
