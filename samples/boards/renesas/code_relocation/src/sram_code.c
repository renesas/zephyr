/*
 * Copyright (c) 2022 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

uint32_t var_sram_data = 10U;

/* Only the addresses are printed, to show where the relocated code ended up */
extern void switch_isr_wrapper(void);
extern void z_rx_arch_switch(void);
extern void group_irq_isr(void);
extern void bsp_group_irq_get_table_index(void);

void function_in_sram(void)
{
	printk("Address of %s %p\n", __func__, &function_in_sram);
	printk("Address of var_sram_data %p (%d)\n", &var_sram_data, var_sram_data);
	printk("Address of switch_isr_wrapper %p\n", &switch_isr_wrapper);
	printk("Address of z_rx_arch_switch %p\n", &z_rx_arch_switch);
	printk("Address of group_irq_isr %p\n", &group_irq_isr);
	printk("Address of bsp_group_irq_get_table_index %p\n", &bsp_group_irq_get_table_index);
}
