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
#include <zephyr/devicetree.h>
#include <soc.h>
#include <bsp_api.h>
#include <zephyr/drivers/clock_control/renesas_rx_cgc.h>

/* itcm and itcm_ns are aliases of the same physical LRAM0 — only one may be
 * enabled at a time. Likewise for dtcm / dtcm_ns (same physical SRAM region).
 */
BUILD_ASSERT(!(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(itcm)) &&
	       DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(itcm_ns))),
	     "itcm and itcm_ns cannot both be enabled: they alias the same physical LRAM0");

BUILD_ASSERT(!(DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(dtcm)) &&
	       DT_NODE_HAS_STATUS_OKAY(DT_NODELABEL(dtcm_ns))),
	     "dtcm and dtcm_ns cannot both be enabled: they alias the same physical SRAM region");

uint32_t SystemCoreClock BSP_SECTION_EARLY_INIT;
volatile uint32_t g_protect_pfswe_counter BSP_SECTION_EARLY_INIT;

void soc_prep_hook(void)
{
#if IS_ENABLED(CONFIG_RX74M_LRAM0_NS)
	/*
	 * LRMSABAR0 resets to LRMSABA = 111111b, making the entire LRAM0 a
	 * secure-only region. The non-secure CPU alias at 0x00000000 (itcm_ns)
	 * is inaccessible until LRMSABA is cleared to 000000b. This must happen
	 * before arch_data_copy() copies the .itcm section to 0x00000000.
	 * Writing LRMSABAR0 requires PRCR_S.PRC4 to be unlocked first (key 0xA5).
	 */
	R_SYSTEM->PRCR_S = (uint16_t)((0xA5u << 8) | BIT(4));
	R_CPSCU->LRMSABAR0 = 0U;
	R_SYSTEM->PRCR_S = (uint16_t)(0xA5u << 8); /* re-lock */
#endif
}

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

	R_SYSTEM->PRCR_S = (uint16_t)((0xA5u << 8) | BIT_U16(1) | BIT_U16(5));

	R_SYSTEM->SYRSTMSK0_b.SWMASK = 0u;
	R_SYSTEM->SWRR_b.SWRR = 0xA501;

	irq_unlock(key);
}
