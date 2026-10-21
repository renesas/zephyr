/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rh850_pic

#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <r_pic_api.h>
#include <r_pic.h>

/* Builds g_pic_cfg / g_pic_ext_cfg from devicetree and runs the build-time checks. */
#include "pic_renesas_rh850_cfg.h"

LOG_MODULE_REGISTER(pic_rh850, CONFIG_RENESAS_PIC_LOG_LEVEL);

/* Only when the PIC node is okay: g_pic_cfg exists only then (pic_renesas_rh850_cfg.h). */
#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rh850_pic)

struct pic_rh850_config {
	const pic_api_t *pic_api;
	const pic_cfg_t *pic_cfg;
};

struct pic_rh850_data {
	pic_instance_ctrl_t pic_ctrl; /* FSP control block, written by R_PIC_Open() */
};

/* PIC is only opened once. */
static int pic_rh850_init(const struct device *dev)
{
	const struct pic_rh850_config *config = dev->config;
	struct pic_rh850_data *data = dev->data;
	fsp_err_t err;

	err = config->pic_api->open(&data->pic_ctrl, config->pic_cfg);
	if (err != FSP_SUCCESS) {
		LOG_ERR("%s: R_PIC_Open failed: %d", dev->name, (int)err);
		return -EIO;
	}

	return 0;
}

#define RH850_PIC_DEVICE_DEFINE(n)                                                                 \
	static struct pic_rh850_data pic_rh850_data_##n;                                           \
	static const struct pic_rh850_config pic_rh850_config_##n = {                              \
		.pic_api = &g_pic_on_pic,                                                          \
		.pic_cfg = &g_pic_cfg,                                                             \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, pic_rh850_init, NULL, &pic_rh850_data_##n, &pic_rh850_config_##n, \
			      POST_KERNEL, CONFIG_RENESAS_PIC_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RH850_PIC_DEVICE_DEFINE)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(renesas_rh850_pic) */
