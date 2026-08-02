/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#define USER_NODE DT_PATH(zephyr_user)

LOG_MODULE_REGISTER(board_control, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(ssi5)) && defined(CONFIG_I2S)
static int i2s_init(void)
{
	const struct gpio_dt_spec audio_codec_gpios =
		GPIO_DT_SPEC_GET_OR(USER_NODE, audio_codec_gpios, {});

	if (!gpio_is_ready_dt(&audio_codec_gpios)) {
		LOG_ERR("I2S control is not ready");
		return -ENODEV;
	}

	if (strcmp(DT_PROP(USER_NODE, audio_codec_dir), "out") == 0) {
		if (gpio_pin_configure_dt(&audio_codec_gpios, GPIO_OUTPUT_LOW)) {
			LOG_ERR("Failed to configure I2S control");
			return -EIO;
		}
	} else if (strcmp(DT_PROP(USER_NODE, audio_codec_dir), "in") == 0) {
		if (gpio_pin_configure_dt(&audio_codec_gpios, GPIO_OUTPUT_HIGH)) {
			LOG_ERR("Failed to configure I2S control");
			return -EIO;
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

SYS_INIT(i2s_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
#endif
