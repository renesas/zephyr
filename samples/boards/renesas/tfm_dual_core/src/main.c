/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

int main(void)
{
	uint32_t count = 0U;

	printk("Hello from CM33 non-secure: %s\n", CONFIG_BOARD_TARGET);

	while (true) {
		k_msleep(1000);
		count++;
		printk("CM33 tick %u\n", count);
	}

	return 0;
}
