/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/cache.h>
#include <zephyr/sys/util.h>
#include <bsp_api.h>

/* 32 KB cache / 32-byte lines = 1024 lines; 100 us >> 1024 cycles at 600 MHz max CPUCLK0 */
#define CACHE_FLUSH_TIMEOUT_US 1000U

void arch_dcache_enable(void)
{
	R_CACHE->OACACTL_b.EN = 1;
	R_CACHE->OACAWTA_b.WT = 1;
	R_CACHE->OACAFCT_b.FL = 1;

	WAIT_FOR(R_CACHE->OACAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL);
}

void arch_dcache_disable(void)
{
	R_CACHE->OACAFCT_b.FL = 1;
	R_CACHE->OACACTL_b.EN = 0;
}

int arch_dcache_flush_all(void)
{
	R_CACHE->OACAFCT_b.FL = 1;

	if (!WAIT_FOR(R_CACHE->OACAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

int arch_dcache_flush_and_invd_all(void)
{
	R_CACHE->OACAFCT_b.FL = 1;

	if (!WAIT_FOR(R_CACHE->OACAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

int arch_dcache_invd_all(void)
{
	return arch_dcache_flush_and_invd_all();
}

int arch_dcache_flush_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	return arch_dcache_flush_and_invd_all();
}

int arch_dcache_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	return arch_dcache_flush_and_invd_all();
}

int arch_dcache_flush_and_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	return arch_dcache_flush_and_invd_all();
}

void arch_icache_enable(void)
{
	R_CACHE->IFCACTL_b.EN = 1;
	R_CACHE->IFCACTL_b.FL = 1;

	WAIT_FOR(R_CACHE->IFCAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL);
}

void arch_icache_disable(void)
{
	R_CACHE->IFCACTL_b.EN = 0;
}

int arch_icache_flush_all(void)
{
	R_CACHE->IFCAFCT_b.FL = 1;

	if (!WAIT_FOR(R_CACHE->IFCAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

int arch_icache_invd_all(void)
{
	R_CACHE->IFCAFCT_b.FL = 1;

	if (!WAIT_FOR(R_CACHE->IFCAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

int arch_icache_flush_and_invd_all(void)
{
	R_CACHE->IFCAFCT_b.FL = 1;

	if (!WAIT_FOR(R_CACHE->IFCAFCT_b.FL == 0, CACHE_FLUSH_TIMEOUT_US, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

int arch_icache_flush_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	return arch_icache_flush_and_invd_all();
}

int arch_icache_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	return arch_icache_flush_and_invd_all();
}

int arch_icache_flush_and_invd_range(void *start_addr, size_t size)
{
	ARG_UNUSED(start_addr);
	ARG_UNUSED(size);
	return arch_icache_flush_and_invd_all();
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
