/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief System/hardware module for RX SOC family
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <soc.h>
#include <bsp_api.h>

uint32_t SystemCoreClock BSP_SECTION_EARLY_INIT;
volatile uint32_t g_protect_pfswe_counter BSP_SECTION_EARLY_INIT;

extern
/**
 * @brief Perform basic hardware initialization at boot.
 *
 * This needs to be run from the very beginning.
 * So the init priority has to be 0 (zero).
 *
 * @return 0
 */
void soc_early_init_hook(void)
{
	SystemCoreClock = BSP_MOCO_HZ;
	g_protect_pfswe_counter = 0;
	renesas_rx_register_protect_open();
}

static inline uint16_t BIT_U16(unsigned n)
{
	return (uint16_t)(1u << n);
}

void sys_arch_reboot(int type)
{
	ARG_UNUSED(type);

	unsigned int key = irq_lock();

	R_SYSTEM->PRCR = (uint16_t)((0xA5u << 8) | BIT_U16(1) | BIT_U16(5));

	R_SYSTEM->SYRSTMSK0_b.SWMASK = 0u;
	R_SYSTEM->SWRR_b.SWRR = 0xA501;

	irq_unlock(key);
}
