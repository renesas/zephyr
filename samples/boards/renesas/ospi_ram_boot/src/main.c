/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/reboot.h>

extern void _start();
extern uint8_t __exvectors_start[];
extern uint8_t rvectors_start[];

/* Main entry point */
int main(void)
{
	printk("Launching primary slot application on %s\n", CONFIG_BOARD);
	printk("Address of Kernel _start function %p\n", (void *)_start);
	printk("Address of main %p\n", (int *)main);
	printk("Address of EXVECTORS section: start %p\n", (uint8_t *)__exvectors_start);
	printk("Address of relocatable vector table section: start %p\n", (uint8_t *)rvectors_start);

	return 0;
}
