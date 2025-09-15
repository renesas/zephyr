/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_control, CONFIG_LOG_DEFAULT_LEVEL);

#if DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(mipi_dsi)) && defined(CONFIG_MIPI_DSI)
static int mipi_dphy_init(void)
{
	const struct gpio_dt_spec mipi_dphy_en_gpios =
		GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), mipi_dphy_en_gpios);

	if (!gpio_is_ready_dt(&mipi_dphy_en_gpios)) {
		LOG_ERR("MIPI D-PHY enable gpio not ready");
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&mipi_dphy_en_gpios, GPIO_OUTPUT_ACTIVE)) {
		LOG_ERR("Failed to configure MIPI D-PHY enable gpio");
		return -EIO;
	}

	return 0;
}

SYS_INIT(mipi_dphy_init, POST_KERNEL, CONFIG_MIPI_DSI_INIT_PRIORITY);
#endif
