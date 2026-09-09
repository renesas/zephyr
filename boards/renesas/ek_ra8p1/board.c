/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_control, CONFIG_LOG_DEFAULT_LEVEL);

#if IS_ENABLED(CONFIG_BOARD_EK_RA8P1_MIPI_CSI2_INIT)

/*
 * Enable the MIPI D-PHY analog block (P108, active-low). Without it,
 * the D-PHY receiver never detects CSI-2 lane activity.
 */
static int mipi_dphy_enable_init(void)
{
	const struct device *ioport1 = DEVICE_DT_GET(DT_NODELABEL(ioport1));
	int ret;

	if (!device_is_ready(ioport1)) {
		LOG_ERR("ioport1 not ready, cannot enable MIPI D-PHY");
		return -ENODEV;
	}

	ret = gpio_pin_configure(ioport1, 8, GPIO_OUTPUT_ACTIVE | GPIO_ACTIVE_LOW);
	if (ret < 0) {
		LOG_ERR("Failed to enable MIPI D-PHY (ioport1.8): %d", ret);
		return ret;
	}

	return 0;
}

SYS_INIT(mipi_dphy_enable_init, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/*
 * Camera connector routing mux (SW4-6/MIPI_SEL) on the on-board
 * PI4IOE5V6408 I/O expander. When the sw4_ioexp node is present, drive
 * MIPI_SEL high to route the camera connector to the MIPI CSI-2 interface,
 * matching the Renesas reference design. Without this setting, the CSI-2
 * receiver never sees the sensor signal.
 */
#if DT_NODE_EXISTS(DT_NODELABEL(sw4_ioexp))

#define SW4_MIPI_SEL_PIN 5

static int mipi_csi_connector_mux_init(void)
{
	const struct device *ioexp = DEVICE_DT_GET(DT_NODELABEL(sw4_ioexp));
	int ret;

	if (!device_is_ready(ioexp)) {
		LOG_ERR("SW4 I/O expander not ready, cannot select MIPI CSI-2 routing");
		return -ENODEV;
	}

	ret = gpio_pin_configure(ioexp, SW4_MIPI_SEL_PIN, GPIO_OUTPUT_HIGH);
	if (ret < 0) {
		LOG_ERR("Failed to select MIPI CSI-2 routing (SW4-6): %d", ret);
		return ret;
	}

	return 0;
}

/*
 * Must run after the expander's own driver (CONFIG_GPIO_PI4IOE5V6408_INIT_PRIORITY,
 * default 70) and the I2C bus it sits on have both initialized.
 */
SYS_INIT(mipi_csi_connector_mux_init, POST_KERNEL, 80);

#endif /* DT_NODE_EXISTS(DT_NODELABEL(sw4_ioexp)) */

#endif /* CONFIG_BOARD_EK_RA8P1_MIPI_CSI2_INIT */
