/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/linker/linker-defs.h>
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

#define TCM_ADDR_ALIGNED_8K 0xFFFFE000U
#define TCM_SIZE_32K        (0x6U << 2)
#define TCM_ENABLE          (0x1u << 0)
#define TCM_BANK_SIZE       KB(32)

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
	/* Enable instruction cache */
	sys_cache_instr_enable();

	/* Enable data cache */
	sys_cache_data_enable();

	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_itcm))
#define ITCM_NODE DT_CHOSEN(zephyr_itcm)
BUILD_ASSERT(DT_REG_SIZE(ITCM_NODE) == 2U * TCM_BANK_SIZE, "ITCM is ATCM + CTCM (64 KB)");
BUILD_ASSERT((DT_REG_ADDR(ITCM_NODE) % TCM_BANK_SIZE) == 0U, "TCM base must be size aligned");
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_dtcm))
#define DTCM_NODE DT_CHOSEN(zephyr_dtcm)
BUILD_ASSERT(DT_REG_SIZE(DTCM_NODE) == TCM_BANK_SIZE, "DTCM is the BTCM (32 KB)");
BUILD_ASSERT((DT_REG_ADDR(DTCM_NODE) % TCM_BANK_SIZE) == 0U, "TCM base must be size aligned");
#endif

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_itcm)) ||                                             \
	DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_dtcm))
/*
 * With ECC, a TCM chunk must be written whole by an aligned 64-bit store before
 * any narrower access (Cortex-R52 TRM, "Preloading TCMs with ECC").
 */
static ALWAYS_INLINE void tcm_zero(uintptr_t base, size_t size)
{
	volatile uint64_t *dst = (volatile uint64_t *)base;

	for (size_t i = 0; i < size / sizeof(uint64_t); i++) {
		dst[i] = 0U;
	}
}

/* Byte loop: memcpy() may itself be placed in the ITCM, which is not loaded yet */
static ALWAYS_INLINE void tcm_load(char *dst, const char *src, size_t len)
{
	volatile char *to = dst;
	const volatile char *from = src;

	if (IS_ENABLED(CONFIG_XIP)) {
		/* arch_data_copy() loads the TCM sections */
		return;
	}

	for (size_t i = 0; i < len; i++) {
		to[i] = from[i];
	}
}
#endif

/*
 * Runs before z_prep_c(): map the TCMs, preload them and load .itcm and
 * .dtcm_data, so that arch_bss_zero() and all later code can use them.
 */
void soc_reset_hook(void)
{
#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_itcm))
	/* Configure TCM: ATCM and CTCM back to back hold the ITCM */
	uint32_t itcm_addr_aligned = DT_REG_ADDR(ITCM_NODE) & TCM_ADDR_ALIGNED_8K;
	uint32_t atcm = itcm_addr_aligned | TCM_SIZE_32K | TCM_ENABLE;
	uint32_t ctcm = (itcm_addr_aligned + TCM_BANK_SIZE) | TCM_SIZE_32K | TCM_ENABLE;

	__asm__ volatile("mcr p15, 0, %0, c9, c1, 0\n" : : "r"(atcm)); /* IMP_ATCMREGIONR */
	__asm__ volatile("mcr p15, 0, %0, c9, c1, 2\n" : : "r"(ctcm)); /* IMP_CTCMREGIONR */
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_dtcm))
	/* Configure TCM: BTCM holds the DTCM */
	uint32_t dtcm_addr_aligned = DT_REG_ADDR(DTCM_NODE) & TCM_ADDR_ALIGNED_8K;
	uint32_t value = dtcm_addr_aligned | TCM_SIZE_32K | TCM_ENABLE;

	__asm__ volatile("mcr p15, 0, %0, c9, c1, 1\n" : : "r"(value)); /* IMP_BTCMREGIONR */
#endif
	barrier_dsync_fence_full();
	barrier_isync_fence_full();

#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_itcm))
	tcm_zero(DT_REG_ADDR(ITCM_NODE), DT_REG_SIZE(ITCM_NODE));
	tcm_load(__itcm_start, __itcm_load_start, (size_t)(__itcm_end - __itcm_start));
#endif
#if DT_NODE_HAS_STATUS_OKAY(DT_CHOSEN(zephyr_dtcm))
	tcm_zero(DT_REG_ADDR(DTCM_NODE), DT_REG_SIZE(DTCM_NODE));
	tcm_load(__dtcm_data_start, __dtcm_data_load_start,
		 (size_t)(__dtcm_data_end - __dtcm_data_start));
#endif
	barrier_dsync_fence_full();
	barrier_isync_fence_full();
}
