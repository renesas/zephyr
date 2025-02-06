/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Local Includes */
#include <zephyr/sys/util.h>
#include <zephyr/drivers/misc/renesas_lvd/renesas_rx_lvd.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if DT_HAS_COMPAT_STATUS_OKAY(renesas_rx_lvd)
#define LVD_NODE DT_INST(0, renesas_rx_lvd)
#endif

#define LVD_ACTION       DT_PROP(LVD_NODE, lvd_action)
#define voltage_theshold DT_PROP(LVD_NODE, voltage_level)

/* In case the action is a maskable interrupt. */
#if LVD_ACTION == 2
#define LVD_INT 1
#else
#define LVD_INT 0
#endif

#if LVD_INT
/* User callback function */
void user_callback(void *user_data)
{
	ARG_UNUSED(user_data);
	printk("LVD: A voltage level change has been detected. \n"
	       "The voltage has exceeded the %dmV threshold. Please be cautious. \n",
	       voltage_theshold);
}
#endif

int main(void)
{
	int ret;
	lvd_status_cross_t status_cross = LVD_STATUS_CROSS_NONE;
	lvd_status_position_t status_position;

	const struct device *const lvd_dev = DEVICE_DT_GET(LVD_NODE);
	if (!device_is_ready(lvd_dev)) {
		printk("LVD: Device not ready.\n");
		return -EINVAL;
	}

#if LVD_INT
	/* Register the user callback function in case the lvd-action was set to maskable interrupt
	 */
	int err = renesas_rx_lvd_register_callback(lvd_dev, user_callback, NULL);
	if (err != 0) {
		printk("Failed to register callback\n");
		return -EINVAL;
	}
#endif

	while (status_cross == LVD_STATUS_CROSS_NONE) {
		printk("The detection voltage level is set to %dV. \n", voltage_theshold);

		/* Get status */
		ret = renesas_rx_lvd_get_status(lvd_dev, &status_position, &status_cross);
		if (ret != 0) {
			printk("Failed to get LVD status\n");
			return -EINVAL;
		}

		switch (status_position) {
		case LVD_STATUS_POSITION_ABOVE:
			printk("LVD: Voltage above detection level\n");
			break;
		case LVD_STATUS_POSITION_BELOW:
			printk("LVD: Voltage below detection level\n");
			break;
		default:
			printk("LVD: Unknown voltage detection status\n");
			break;
		}

		printk("LVD: No voltage cross detection\n");
		k_sleep(K_MSEC(1000));
	}

	printk("LVD: Voltage crossed detection level\n");

	ret = renesas_rx_lvd_clear_status(lvd_dev);
	if (ret != 0) {
		printk("Failed to clear LVD status\n");
		return -EINVAL;
	}

	return 0;
}
