/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <string.h>
#include <wchar.h>

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/spinlock.h>
#include <zephyr/toolchain.h>
#include <zephyr/arch/rx/mpu.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/dt-bindings/memory-attr/memory-attr.h>

#if defined(CONFIG_MEM_ATTR)
#include <zephyr/mem_mgmt/mem_attr.h>
#endif

/*
 * RX CPU MPU notes (RX74M HUM section 17):
 *
 *  - 8 access-control regions, 32-byte page granularity.
 *  - Each region is described by a start page (RSPAGEn: DCA[1:0], ICA[2],
 *    RSPN[31:5]) and an end page (REPAGEn: V[0], UAC[3:1], REPN[31:5]).
 *  - The end page is *inclusive*.
 *  - Memory protection only applies in user mode; in supervisor mode only the
 *    cache attribute settings are effective. As a result this MPU can isolate
 *    user threads but cannot protect the kernel from itself (no supervisor
 *    stack guard, no kernel-RO).
 *  - The background region (MPBAC) provides the default cache/access attribute
 *    for any address not covered by an explicit region.
 *  - IMPORTANT: overlapping regions resolve as the *logical OR* of their
 *    attributes, with permission and caching given priority
 *    (write-back > write-through > disabled). The background region is *not*
 *    part of this OR. This means a region cannot be made *more* restrictive by
 *    overlaying it on a broader one. To carve a non-cacheable window (e.g. DMA /
 *    nocache) out of
 *    a cacheable area, the regions must be laid out *non-overlapping* - see
 *    rx_mpu_add_ram_regions() which marks only the nocache window non-cacheable.
 */

/* Access rights for the nocache window: supervisor RWX, user no-access. */
#define RX_MPU_RAM_ACCESS   RX_MPU_ACCESS_P_RWX_U_NA
/* Access rights for Code MRAM: supervisor RWX, user read/execute only. */
#define RX_MPU_FLASH_ACCESS RX_MPU_ACCESS_P_RWX_U_RX

#define RX_MPU_PAGE_SIZE 32U

/* User access-control (UAC / MHITD.UHACD) bit positions, user mode. */
#define RX_MPU_UAC_X BIT(0) /* execute */
#define RX_MPU_UAC_W BIT(1) /* write */
#define RX_MPU_UAC_R BIT(2) /* read */

/* MPBAC.DCA field values (data cache attribute). */
#define RX_MPU_DCA_DISABLED      0U /* not cached */
#define RX_MPU_DCA_WRITE_THROUGH 1U /* write-through */
#define RX_MPU_DCA_WRITE_BACK    3U /* write-back (value 2 is reserved) */

/* MPBAC.ICA field values (instruction cache attribute). */
#define RX_MPU_ICA_DISABLED 0U
#define RX_MPU_ICA_ENABLED  1U

/* MPBAC.UBAC: user-mode access bits, same layout as UAC in RPAGE. */
#define RX_MPU_UBAC_RWX  (RX_MPU_UAC_R | RX_MPU_UAC_W | RX_MPU_UAC_X)
#define RX_MPU_UBAC_NONE 0U

/* log2(RX_MPU_PAGE_SIZE): used to convert byte addresses to page numbers. */
#define RX_MPU_PAGE_SHIFT 5U

/* attr field encoding passed to map_region_set: DCA in bits[1:0], ICA in bit[2]. */
#define RX_MPU_ATTR_DCA_MASK  0x3U
#define RX_MPU_ATTR_ICA_SHIFT 2U

/** MPU foreground map for kernel mode. */
static struct rx_mpu_map rx_mpu_map_fg_kernel;

/** Make sure write to the MPU region / search registers is atomic. */
static struct k_spinlock rx_mpu_lock;

#if defined(CONFIG_USERSPACE)
/** Number of map slots consumed by the static kernel regions. */
static uint8_t rx_mpu_static_regions_num;
#endif

#if CONFIG_BOOTLOADER_MCUBOOT
#define FLASH_BASE (CONFIG_FLASH_BASE_ADDRESS)
#define FLASH_SIZE (CONFIG_FLASH_BASE_ADDRESS + (DT_REG_SIZE(DT_CHOSEN(zephyr_code_partition)) - 1))
#else
#define FLASH_BASE (DT_REG_ADDR(DT_CHOSEN(zephyr_flash)))
#define FLASH_SIZE                                                                                 \
	(DT_REG_ADDR(DT_CHOSEN(zephyr_flash)) + (DT_REG_SIZE(DT_CHOSEN(zephyr_flash)) - 1))
#endif

/*
 * Software-only manipulation of an MPU map. These functions only touch the
 * in-memory map; nothing reaches the hardware until rx_mpu_map_write() is
 * called. This split lets us keep a separate map per memory domain and flush
 * the active one on context switch.
 */

static int map_region_set(struct rx_mpu_map *map, uintptr_t start, uintptr_t end,
			  uint8_t access_rights, uint32_t attr)
{
	if (start >= end) {
		return -EINVAL;
	}

	for (int entry = 0; entry < RX_MPU_NUM_ENTRIES; entry++) {
		if (map->partitions[entry].end == 0) { /* end == 0 means free */
			map->partitions[entry].start = start;
			map->partitions[entry].end = end;
			map->partitions[entry].attr = attr;
			map->partitions[entry].access_rights = access_rights;
			return entry;
		}
	}

	return -ENOMEM;
}

#if defined(CONFIG_USERSPACE)
static int map_region_clear(struct rx_mpu_map *map, uintptr_t start, uintptr_t end)
{
	for (int entry = 0; entry < RX_MPU_NUM_ENTRIES; entry++) {
		if (map->partitions[entry].start == start && map->partitions[entry].end == end) {
			map->partitions[entry] = (struct z_rx_mpu_partition){0};
			return 0;
		}
	}

	return -ENOENT;
}
#endif /* CONFIG_USERSPACE */

/*
 * Flush an entire software map to the 8 hardware regions. Because RX regions
 * are independent (no required ordering, gaps fall through to MPBAC) this is a
 * plain loop with no required ordering.
 */
void rx_mpu_map_write(struct rx_mpu_map *map)
{
	k_spinlock_key_t key = k_spin_lock(&rx_mpu_lock);

	for (int entry = 0; entry < RX_MPU_NUM_ENTRIES; entry++) {
		const struct z_rx_mpu_partition *p = &map->partitions[entry];

		if (p->end == 0) {
			/* Empty slot: invalidate the region. */
			R_MPU->RPAGE[entry].EN_b.V = 0;
			continue;
		}

		R_MPU->RPAGE[entry].ST_b.DCA = (p->attr & RX_MPU_ATTR_DCA_MASK);
		R_MPU->RPAGE[entry].ST_b.ICA = (p->attr >> RX_MPU_ATTR_ICA_SHIFT) & 1U;
		R_MPU->RPAGE[entry].ST_b.RSPN = p->start >> RX_MPU_PAGE_SHIFT;

		R_MPU->RPAGE[entry].EN_b.UAC = p->access_rights;
		R_MPU->RPAGE[entry].EN_b.REPN = (p->end - 1) >> RX_MPU_PAGE_SHIFT;
		R_MPU->RPAGE[entry].EN_b.V = 1;                    /* enables the region */
	}

	k_spin_unlock(&rx_mpu_lock, key);
}

/*
 * Configure the background region (MPBAC): the default applied to any address
 * not covered by an explicit region. Mark the entire address space cacheable
 * (write-back data cache + instruction cache) and grant user mode full RWX
 * access. This means all memory not matched by an explicit MPU region is
 * treated as cacheable and fully accessible. Peripheral / MMIO ranges that
 * must not be cached should be covered by explicit regions with cache disabled.
 */
static void rx_mpu_set_background(void)
{
	k_spinlock_key_t key = k_spin_lock(&rx_mpu_lock);

	R_MPU->MPBAC_b.DCA = RX_MPU_DCA_WRITE_BACK;
	R_MPU->MPBAC_b.ICA = RX_MPU_ICA_ENABLED;
	R_MPU->MPBAC_b.UBAC = RX_MPU_UBAC_RWX;

	k_spin_unlock(&rx_mpu_lock, key);
}

/*
 * With MPBAC set cacheable, all SRAM is cacheable by default without any
 * explicit region. This function only needs to override MPBAC for the
 * nocache window by adding a single non-cacheable region for that range.
 * No split-around-hole pattern is needed.
 */
static void rx_mpu_add_ram_regions(struct rx_mpu_map *map)
{
#if defined(CONFIG_NOCACHE_MEMORY)
	const uintptr_t nc_start = (uintptr_t)_nocache_ram_start;
	const uintptr_t nc_end = (uintptr_t)_nocache_ram_end;

	if (nc_end > nc_start) {
		map_region_set(map, nc_start, nc_end, RX_MPU_RAM_ACCESS, RX_MPU_CACHE_ALL_DISABLED);
	}
#endif /* CONFIG_NOCACHE_MEMORY */

#if defined(CONFIG_ARCH_HAS_RAMFUNC_SUPPORT)
	/*
	 * Code executed from SRAM (ramfunc) needs user-mode execute permission.
	 * MPBAC already provides cacheable background; this explicit region adds
	 * user-mode execute on top. Overlaying more permission with identical
	 * caching is safe under the OR rule.
	 */
	if ((uintptr_t)__ramfunc_end > (uintptr_t)__ramfunc_start) {
		map_region_set(map, (uintptr_t)__ramfunc_start, (uintptr_t)__ramfunc_end,
			       RX_MPU_ACCESS_P_RWX_U_RX, RX_MPU_CACHE_ALL_ENABLED);
	}
#endif /* CONFIG_ARCH_HAS_RAMFUNC_SUPPORT */
}

#if defined(CONFIG_MEM_ATTR)
/*
 * Translate the generic devicetree memory attributes (zephyr,memory-attr) into
 * RX cache / access settings and program a region for every memory node that
 * carries them. This is the "configure memory via DTS macro" path.
 *
 * Supported generic attributes (see dt-bindings/memory-attr/memory-attr.h):
 *   DT_MEM_CACHEABLE    -> instruction + data cache enabled (write-back)
 *   (absent)            -> cache disabled (e.g. DMA buffers, MMIO RAM)
 *   DT_MEM_NON_VOLATILE -> read/execute only in user mode (flash-like)
 *   (absent)            -> read/write in user mode
 *
 * Note: DT non-cacheable regions are expressed as explicit RPAGE entries.
 * Since MPBAC is background-only and not part of the explicit-region OR,
 * a non-cacheable DT region correctly overrides the cacheable MPBAC background
 * for that address range. DT regions are intended for memory outside the
 * default SRAM / flash nodes (e.g. external PSRAM / xSPI mapped memory).
 */
static void rx_mpu_configure_regions_from_dt(struct rx_mpu_map *map)
{
	const struct mem_attr_region_t *regions;
	size_t num_regions = mem_attr_get_regions(&regions);

	for (size_t idx = 0; idx < num_regions; idx++) {
		const uint32_t dt_attr = regions[idx].dt_attr;
		uint32_t cache_attr;
		uint8_t access;

		/* Only honour nodes that actually carry a generic attribute. */
		if (DT_MEM_ATTR_GET(dt_attr) == 0) {
			continue;
		}

		cache_attr = (dt_attr & DT_MEM_CACHEABLE) ? RX_MPU_CACHE_ALL_ENABLED
							  : RX_MPU_CACHE_ALL_DISABLED;
		access = (dt_attr & DT_MEM_NON_VOLATILE) ? RX_MPU_FLASH_ACCESS
							 : RX_MPU_ACCESS_P_RWX_U_RW;

		if (map_region_set(map, regions[idx].dt_addr,
				   regions[idx].dt_addr + regions[idx].dt_size, access,
				   cache_attr) < 0) {
			__ASSERT(false, "RX MPU: out of regions for DT node %s",
				 regions[idx].dt_name);
		}
	}
}
#endif /* CONFIG_MEM_ATTR */

void rx_mpu_init(void)
{
	/* Background (MPBAC): cacheable write-back for all unmapped addresses,
	 * user mode RWX. Explicit regions override this for nocache windows.
	 */
	rx_mpu_set_background();

	/*
	 * Code MRAM (zephyr,flash): cache is already provided by MPBAC.
	 * This region exists solely to restrict user mode to read/execute only.
	 */
	map_region_set(&rx_mpu_map_fg_kernel, FLASH_BASE, FLASH_SIZE, RX_MPU_FLASH_ACCESS,
		       RX_MPU_CACHE_ALL_ENABLED);

	/* Override MPBAC cacheable background for any nocache window in SRAM. */
	rx_mpu_add_ram_regions(&rx_mpu_map_fg_kernel);

#if defined(CONFIG_MEM_ATTR)
	/* Additional regions described in the devicetree. */
	rx_mpu_configure_regions_from_dt(&rx_mpu_map_fg_kernel);
#endif

#if defined(CONFIG_USERSPACE)
	/* Remember how many slots the static kernel layout used so that memory
	 * domains know how many regions remain for dynamic partitions.
	 */
	rx_mpu_static_regions_num = 0;
	for (int entry = 0; entry < RX_MPU_NUM_ENTRIES; entry++) {
		if (rx_mpu_map_fg_kernel.partitions[entry].end != 0) {
			rx_mpu_static_regions_num++;
		}
	}
#endif

	/* Program the foreground kernel map into hardware and enable the MPU. */
	rx_mpu_map_write(&rx_mpu_map_fg_kernel);

	R_MPU->MPEN = 1; /* Enable MPU map */
}

#if defined(CONFIG_USERSPACE)
/*
 * Userspace / memory-domain support.
 *
 * Model:
 *  - Each k_mem_domain owns a full software map (static kernel regions copied
 *    from the foreground kernel map, plus one region per partition).
 *  - MPBAC grants user RWX by default, so partitions refine access by adding
 *    explicit regions with tighter permissions. Revoking a partition simply
 *    clears its slot; unmapped addresses fall back to MPBAC (user RWX).
 *  - On context switch the active thread's map is flushed with
 *    rx_mpu_map_write().
 *
 * NOTE: full CONFIG_USERSPACE also requires the RX privilege-switching and
 * syscall machinery (ARCH_HAS_USERSPACE), which is not yet in place; these
 * hooks are the MPU half of that work.
 */

int arch_mem_domain_init(struct k_mem_domain *domain)
{
	/* Start from the kernel foreground map (static flash / SRAM regions). */
	domain->arch.mpu_map = rx_mpu_map_fg_kernel;

	return 0;
}

int arch_mem_domain_max_partitions_get(void)
{
	/* One region per partition, minus the slot reserved for the thread stack. */
	return RX_MPU_NUM_ENTRIES - rx_mpu_static_regions_num - 1;
}

int arch_mem_domain_partition_add(struct k_mem_domain *domain, uint32_t partition_id)
{
	struct k_mem_partition *partition = &domain->partitions[partition_id];
	uintptr_t end_addr = partition->start + partition->size;
	int ret;

	if (end_addr <= partition->start) {
		return -EINVAL;
	}

	ret = map_region_set(&domain->arch.mpu_map, partition->start, end_addr,
			     (uint8_t)partition->attr, RX_MPU_CACHE_ALL_ENABLED);
	if (ret < 0) {
		return ret; /* -ENOMEM: out of MPU regions */
	}

	if (_current->mem_domain_info.mem_domain == domain) {
		rx_mpu_map_write(_current->arch.mpu_map);
	}

	return 0;
}

int arch_mem_domain_partition_remove(struct k_mem_domain *domain, uint32_t partition_id)
{
	struct k_mem_partition *partition = &domain->partitions[partition_id];
	uintptr_t end_addr = partition->start + partition->size;

	if (end_addr <= partition->start) {
		return -EINVAL;
	}

	/*
	 * Dropping the explicit region reverts this address range to the MPBAC
	 * background (user RWX). This is intentional: partitions grant specific
	 * access on top of the default; revoking returns to the default.
	 */
	(void)map_region_clear(&domain->arch.mpu_map, partition->start, end_addr);

	if (_current->mem_domain_info.mem_domain == domain) {
		rx_mpu_map_write(_current->arch.mpu_map);
	}

	return 0;
}

int arch_mem_domain_thread_add(struct k_thread *thread)
{
	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;
	uintptr_t stack_start = thread->stack_info.start;
	uintptr_t stack_end = stack_start + thread->stack_info.size;
	int ret = 0;

	thread->arch.mpu_map = &domain->arch.mpu_map;

	/*
	 * Grant the user thread access to its own stack. For non-migration
	 * cases this is (re)done by rx_user_stack_perms() when the thread drops
	 * to user mode; keeping it here makes migration self-contained.
	 */
	if ((thread->base.user_options & K_USER) != 0) {
		ret = map_region_set(&domain->arch.mpu_map, stack_start, stack_end,
				     RX_MPU_ACCESS_P_RWX_U_RW, RX_MPU_CACHE_ALL_ENABLED);
		ret = (ret < 0) ? ret : 0;
	}

	if (thread == _current) {
		rx_mpu_map_write(thread->arch.mpu_map);
	}

	return ret;
}

int arch_mem_domain_thread_remove(struct k_thread *thread)
{
	struct k_mem_domain *domain = thread->mem_domain_info.mem_domain;
	uintptr_t stack_start = thread->stack_info.start;
	uintptr_t stack_end = stack_start + thread->stack_info.size;

	if ((thread->base.user_options & K_USER) == 0) {
		return 0;
	}

	/* Revoke the stack grant (meaningful on real removal / abort). */
	(void)map_region_clear(&domain->arch.mpu_map, stack_start, stack_end);

	if (thread == _current) {
		rx_mpu_map_write(thread->arch.mpu_map);
	}

	return 0;
}

/*
 * Region Search: write the address to MPSA, trigger a search via MPOPS.S,
 * then read the OR'd user access-control bits of the hit region from
 * MHITD.UHACD. Cache attributes are not reported, which is fine for
 * access validation.
 */
static uint8_t rx_region_search_data_uac(uintptr_t addr)
{
	uint8_t uac;
	k_spinlock_key_t key = k_spin_lock(&rx_mpu_lock);

	R_MPU->MPSA = addr;
	R_MPU->MPOPS_b.S = 1;
	while (R_MPU->MPOPS_b.S != 0) {
		/* Wait for the search to complete. */
	}
	uac = R_MPU->MHITD_b.UHACD;

	k_spin_unlock(&rx_mpu_lock, key);

	return uac;
}

int arch_buffer_validate(const void *addr, size_t size, int write)
{
	uintptr_t a = ROUND_DOWN((uintptr_t)addr, RX_MPU_PAGE_SIZE);
	uintptr_t last = ROUND_UP((uintptr_t)addr + size, RX_MPU_PAGE_SIZE);
	uint8_t need = (write != 0) ? RX_MPU_UAC_W : RX_MPU_UAC_R;

	for (; a < last; a += RX_MPU_PAGE_SIZE) {
		if ((rx_region_search_data_uac(a) & need) != need) {
			return -EPERM;
		}
	}

	return 0;
}

void rx_user_stack_perms(struct k_thread *thread)
{
	uintptr_t stack_start = thread->stack_info.start;
	uintptr_t stack_end = stack_start + thread->stack_info.size;
	int ret;

	(void)memset((void *)stack_start, (IS_ENABLED(CONFIG_INIT_STACKS)) ? 0xAA : 0x00,
		     thread->stack_info.size - thread->stack_info.delta);

	ret = map_region_set(thread->arch.mpu_map, stack_start, stack_end, RX_MPU_ACCESS_P_RWX_U_RW,
			     RX_MPU_CACHE_ALL_ENABLED);
	__ASSERT_NO_MSG(ret >= 0);
	ARG_UNUSED(ret);

	rx_mpu_map_write(thread->arch.mpu_map);
}

#endif /* CONFIG_USERSPACE */
