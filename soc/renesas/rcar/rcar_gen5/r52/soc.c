/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>
#include <stdint.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

/* Support for Power Domains */
#define MDLC13_BASE         0xC9C90000u
#define MDLC13_MAX_PDID_NUM 6u

#define MDLC_PKCPROT0_OFF 0x0CF0u
#define MDLC_MPIER0_OFF   0x0110u
#define MDLC_MPIMR0_OFF   0x0120u
#define MDLC_MPDG_OFF     0x0200u
#define MDLC_MPDGS_OFF    0x0300u

#define MDLC_PKCPROT_UNLOCK 0xA5A5A501u
#define MDLC_PKCPROT_LOCK   0xA5A5A500u

#define MDLC_POWER_STATE_1    0x00000001u
#define MDLC_POWER_STATE_RUN  0x00000003u
#define MDLC_POWER_STATE_MASK 0x00000003u

/* Wait for power status to change */
static int mdlc_wait_power_status(uintptr_t base, uint32_t pdid, uint32_t expected)
{
	uintptr_t mpdgs = base + MDLC_MPDGS_OFF + (pdid * 4u);

	for (uint32_t timeout = 0; timeout < 10000u; timeout++) {
		uint32_t val = sys_read32(mpdgs);

		if ((val & MDLC_POWER_STATE_MASK) == expected) {
			return 0;
		}

		k_busy_wait(20);
	}

	return -ETIMEDOUT;
}

/* Set power gating to RUN */
__maybe_unused static int mdlc_module_power_gating(uintptr_t base, uint32_t max_pdid)
{
	uintptr_t pkcprot0 = base + MDLC_PKCPROT0_OFF;
	uintptr_t mpier0 = base + MDLC_MPIER0_OFF;
	uintptr_t mpimr0 = base + MDLC_MPIMR0_OFF;

	sys_write32(MDLC_PKCPROT_UNLOCK, pkcprot0);

	for (uint32_t pdid = 0; pdid <= max_pdid; pdid++) {
		uintptr_t mpdg = base + MDLC_MPDG_OFF + (pdid * 4u);
		uintptr_t mpdgs = base + MDLC_MPDGS_OFF + (pdid * 4u);

		if ((sys_read32(mpdgs) & MDLC_POWER_STATE_MASK) == MDLC_POWER_STATE_RUN) {
			continue;
		}

		sys_write32(0x00000000u, mpier0);
		sys_write32(0x00000001u, mpimr0);

		sys_write32(MDLC_POWER_STATE_1, mpdg);
		if (mdlc_wait_power_status(base, pdid, MDLC_POWER_STATE_1) != 0) {
			sys_write32(MDLC_PKCPROT_LOCK, pkcprot0);
			return -ETIMEDOUT;
		}

		sys_write32(MDLC_POWER_STATE_RUN, mpdg);
		if (mdlc_wait_power_status(base, pdid, MDLC_POWER_STATE_RUN) != 0) {
			sys_write32(MDLC_PKCPROT_LOCK, pkcprot0);
			return -ETIMEDOUT;
		}
	}

	sys_write32(MDLC_PKCPROT_LOCK, pkcprot0);

	return 0;
}

void soc_early_init_hook(void)
{
#if defined(CONFIG_MDIO_RENESAS_RCAR_RSW3)
	/* Set up power domains for RSW3 (Ethernet) modules */
	int ret = mdlc_module_power_gating(MDLC13_BASE, MDLC13_MAX_PDID_NUM);

	if (ret != 0) {
		LOG_ERR("Failed to turn on RSW3 power domains");
	}
#endif
}
