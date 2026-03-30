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
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>

uint32_t SystemCoreClock BSP_SECTION_EARLY_INIT;
volatile uint32_t g_protect_pfswe_counter BSP_SECTION_EARLY_INIT;

void soc_early_init_hook(void)
{
	SystemCoreClock = BSP_MOCO_HZ;
	g_protect_pfswe_counter = 0;
	renesas_rx_register_protect_open();
	
	/* Initialize the system clock and peripheral clock */
	bsp_clock_init();

	FSP_HARDWARE_REGISTER_WAIT(R_SYSTEM->PDCTRESWM_b.PDCSF, 0);

	if (1 == R_SYSTEM->PDCTRESWM_b.PDPGSF) {
		/* Turn on ESWM power domain.
		 * This requires MOCO to be enabled, but MOCO is always enabled after
		 * bsp_clock_init().
		 */
		R_BSP_RegisterProtectDisable(BSP_REG_PROTECT_OM_LPC_BATT_SWR);
		FSP_HARDWARE_REGISTER_WAIT((R_SYSTEM->PDCTRESWM & (R_SYSTEM_PDCTRESWM_PDCSF_Msk |
								   R_SYSTEM_PDCTRESWM_PDPGSF_Msk)),
					   R_SYSTEM_PDCTRESWM_PDPGSF_Msk);
		R_SYSTEM->PDCTRESWM = 0;
		FSP_HARDWARE_REGISTER_WAIT((R_SYSTEM->PDCTRESWM & (R_SYSTEM_PDCTRESWM_PDCSF_Msk |
								   R_SYSTEM_PDCTRESWM_PDPGSF_Msk)),
					   0);
		R_BSP_RegisterProtectEnable(BSP_REG_PROTECT_OM_LPC_BATT_SWR);
	}
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
