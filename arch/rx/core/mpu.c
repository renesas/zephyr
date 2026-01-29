/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/spinlock.h>
#include <zephyr/toolchain.h>
#include <zephyr/arch/rx/mpu.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util_macro.h>

/** MPU foreground map for kernel mode. */
static struct rx_mpu_map rx_mpu_map_fg_kernel;

/** Make sure write to the MPU region is atomic. */
static struct k_spinlock rx_mpu_lock;

extern uint32_t __image_text_start;
extern uint32_t __image_text_end;

static const struct z_rx_mpu_partition static_regions[] = {
#if defined(CONFIG_NOCACHE_MEMORY)
	{
		/* Special non-cacheable RAM area */
		.start = (uint32_t)&_nocache_ram_start,
		.end = (uint32_t)&_nocache_ram_end,
		.attr = RX_MPU_CACHE_ICACHE_DIS_DCACHE_DIS,
		.access_rights = RX_MPU_ACCESS_P_RWX_U_NA,
	},
#endif /* CONFIG_NOCACHE_MEMORY */
#if defined(CONFIG_ARCH_HAS_RAMFUNC_SUPPORT)
	{
		/* Special RAM area for program text */
		.start = (uint32_t)&__ramfunc_start,
		.end = (uint32_t)&__ramfunc_end,
		.attr = RX_MPU_CACHE_ICACHE_EN_DCACHE_WB,
		.access_rights = RX_MPU_ACCESS_P_RWX_U_RX,
	},
#endif /* CONFIG_ARCH_HAS_RAMFUNC_SUPPORT */
	/*
	 * Mark the zephyr execution regions (data, bss, noinit, etc.)
	 * cacheable, read / write and non-executable
	 */
	{
		.start = (uint32_t)&__data_region_start,
		.end = (uint32_t)&_image_ram_end,
		.attr = RX_MPU_CACHE_ICACHE_EN_DCACHE_WB,
		.access_rights = RX_MPU_ACCESS_P_RWX_U_NA,
	},
	{
		/* Mark text segment cacheable, read only and executable */
		.start = (uint32_t)&__image_text_start,
		.end = (uint32_t)&__image_text_end,
		.attr = RX_MPU_CACHE_ICACHE_EN_DCACHE_WB,
		.access_rights = RX_MPU_ACCESS_P_RWX_U_RX,
	},
	{
		/* Mark rodata segment cacheable, read only and non-executable */
		.start = (uint32_t)&__rodata_region_start,
		.end = (uint32_t)&__rodata_region_end,
		.attr = RX_MPU_CACHE_ICACHE_EN_DCACHE_WB,
		.access_rights = RX_MPU_ACCESS_P_RWX_U_R,
	}};

static int rx_mpu_map_region_add(struct rx_mpu_map *map, uintptr_t start, uintptr_t end,
				 uint8_t access_rights, uint32_t attr)
{
	int ret = 0;
	int entry;
	uint32_t start_page, end_page;

	if (start >= end) {
		ret = -EINVAL;
		goto out;
	}

	/* Find a free slot */
	for (entry = 0; entry < RX_MPU_NUM_ENTRIES; entry++) {
		if (map->partitions[entry].end == 0) { /* Assuming end=0 means free */
			break;
		}
	}

	if (entry >= RX_MPU_NUM_ENTRIES) {
		ret = -ENOMEM;
		goto out;
	}

	/* Set the partition */
	map->partitions[entry].start = start;
	map->partitions[entry].end = end;
	map->partitions[entry].attr = attr;
	map->partitions[entry].access_rights = access_rights;
	/* Calculate page numbers (32-byte pages) */
	start_page = start >> 5;
	end_page = (end - 1) >> 5; /* End page is inclusive? Check HW */

	/* Write to HW */
	k_spinlock_key_t key = k_spin_lock(&rx_mpu_lock);

	R_MPU->RPAGE[entry].ST_b.DCA = (attr & 0x3);      /* DCA bits 0-1 */
	R_MPU->RPAGE[entry].ST_b.ICA = (attr >> 2) & 0x1; /* ICA bit 2 */
	R_MPU->RPAGE[entry].ST_b.RSPN = start_page;

	R_MPU->RPAGE[entry].EN_b.V = 1;               /* Valid */
	R_MPU->RPAGE[entry].EN_b.UAC = access_rights; /* Assuming UAC maps to access_rights */
	R_MPU->RPAGE[entry].EN_b.REPN = end_page;

	k_spin_unlock(&rx_mpu_lock, key);

out:
	return ret;
}

void rx_mpu_init(void)
{
	int entry;

	for (entry = 0; entry < ARRAY_SIZE(static_regions); entry++) {
		rx_mpu_map_region_add(&rx_mpu_map_fg_kernel, static_regions[entry].start,
				      static_regions[entry].end,
				      static_regions[entry].access_rights,
				      static_regions[entry].attr);
	}

	R_MPU->MPEN = 1; /* Enable MPU map */
}
