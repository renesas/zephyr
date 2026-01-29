/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/cpu.h>
#include <zephyr/cache.h>
#include <bsp_api.h>

void arch_dcache_enable(void)
{
	R_CACHE->OACACTL_b.EN = 1;
	R_CACHE->OACAWTA_b.WT = 1;
	R_CACHE->OACAFCT_b.FL = 1;

	while (R_CACHE->OACAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
}

void arch_dcache_disable(void)
{
	R_CACHE->OACAFCT_b.FL = 1;
	R_CACHE->OACACTL_b.EN = 0;
}

int arch_dcache_flush_all(void)
{
	R_CACHE->OACAFCT_b.FL = 1;
	while (R_CACHE->OACAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
	return 0;
}

int arch_dcache_invd_all(void)
{
	arch_dcache_flush_and_invd_all();
	return 0;
}

int arch_dcache_flush_and_invd_all(void)
{
	R_CACHE->OACAFCT_b.FL = 1;
	while (R_CACHE->OACAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
	return 0;
}

int arch_dcache_flush_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	arch_dcache_flush_and_invd_all();
	return 0;
}

int arch_dcache_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	arch_dcache_flush_and_invd_all();
	return 0;
}

int arch_dcache_flush_and_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	arch_dcache_flush_and_invd_all();
	return 0;
}

void arch_icache_enable(void)
{
	R_CACHE->IFCACTL_b.EN = 1;
	R_CACHE->IFCACTL_b.FL = 1;

	while (R_CACHE->IFCAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
}

void arch_icache_disable(void)
{
	R_CACHE->IFCACTL_b.EN = 0;
}

int arch_icache_flush_all(void)
{
	R_CACHE->IFCAFCT_b.FL = 1;
	while (R_CACHE->IFCAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
	return 0;
}

int arch_icache_invd_all(void)
{
	R_CACHE->IFCAFCT_b.FL = 1;
	while (R_CACHE->IFCAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
	return 0;
}

int arch_icache_flush_and_invd_all(void)
{
	R_CACHE->IFCAFCT_b.FL = 1;
	while (R_CACHE->IFCAFCT_b.FL != 0)
	{
		/* Wait until cache is enabled and ready */
	}
	return 0;
}

int arch_icache_flush_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	arch_icache_flush_and_invd_all();
	return 0;
}

int arch_icache_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	arch_icache_flush_and_invd_all();
	return 0;
}

int arch_icache_flush_and_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	arch_icache_flush_and_invd_all();

	return 0;
}

void arch_cache_init(void)
{
	/* Enable I-Cache and D-Cache */
#if CONFIG_ICACHE
	arch_icache_enable();
#endif
#if CONFIG_DCACHE
	arch_dcache_enable();
#endif
}
