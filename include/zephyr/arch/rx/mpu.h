/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/toolchain.h>
#include <zephyr/sys/util_macro.h>
#include <bsp_api.h>

#ifndef ZEPHYR_INCLUDE_ARCH_RX_MPU_H
#define ZEPHYR_INCLUDE_ARCH_RX_MPU_H

/* Convenience macros to represent the RX MPU
 * configuration for memory access permission and
 * cache-ability attribution.
 */

#define RX_MPU_NUM_ENTRIES (CONFIG_RX_MPU_NUM_REGIONS)

/**
 * Struct to describe a memory region [start, end).
 */
struct z_rx_mpu_partition {
	/** Start address of the memory region. */
	uint32_t start;
	/** End address of the memory region. */
	uint32_t end;
	/** Access rights and memory type attributes. */
	uint32_t attr;
	uint8_t access_rights;
};

/* P is always RWX, varying U mode permissions (b3:Read, b2:Write, b1:Execute) */
#define RX_MPU_ACCESS_P_RWX_U_NA  (0) /* User: No access */
#define RX_MPU_ACCESS_P_RWX_U_X   (1) /* User: Execute only */
#define RX_MPU_ACCESS_P_RWX_U_W   (2) /* User: Write only */
#define RX_MPU_ACCESS_P_RWX_U_WX  (3) /* User: Write and Execute */
#define RX_MPU_ACCESS_P_RWX_U_R   (4) /* User: Read only */
#define RX_MPU_ACCESS_P_RWX_U_RX  (5) /* User: Read and Execute */
#define RX_MPU_ACCESS_P_RWX_U_RW  (6) /* User: Read and Write */
#define RX_MPU_ACCESS_P_RWX_U_RWX (7) /* User: Read, Write, and Execute */

/* Data Cache Attribute (DCA) - bits [1:0] */
#define RX_MPU_CACHE_DCA_DISABLED      (0 << 0) /* Data cache disabled */
#define RX_MPU_CACHE_DCA_WRITE_THROUGH (1 << 0) /* Data cache enabled (write-through) */
#define RX_MPU_CACHE_DCA_PROHIBITED    (2 << 0) /* Setting prohibited */
#define RX_MPU_CACHE_DCA_WRITE_BACK    (3 << 0) /* Data cache enabled (write-back) */

/* Instruction Cache Attribute (ICA) - bit 2 */
#define RX_MPU_CACHE_ICA_DISABLED (0 << 2) /* Instruction cache disabled */
#define RX_MPU_CACHE_ICA_ENABLED  (1 << 2) /* Instruction cache enabled */

/* Combined cache attribute macros */
#define RX_MPU_CACHE_ATTR(dca, ica) ((dca) | (ica))

/* Convenience macros for common cache configurations */
#define RX_MPU_CACHE_ICACHE_EN_DCACHE_WB  (RX_MPU_CACHE_ICA_ENABLED | RX_MPU_CACHE_DCA_WRITE_BACK)
#define RX_MPU_CACHE_ICACHE_EN_DCACHE_WT  (RX_MPU_CACHE_ICA_ENABLED | RX_MPU_CACHE_DCA_WRITE_THROUGH)
#define RX_MPU_CACHE_ICACHE_EN_DCACHE_DIS (RX_MPU_CACHE_ICA_ENABLED | RX_MPU_CACHE_DCA_DISABLED)
#define RX_MPU_CACHE_ICACHE_DIS_DCACHE_WB (RX_MPU_CACHE_ICA_DISABLED | RX_MPU_CACHE_DCA_WRITE_BACK)
#define RX_MPU_CACHE_ICACHE_DIS_DCACHE_WT                                                          \
	(RX_MPU_CACHE_ICA_DISABLED | RX_MPU_CACHE_DCA_WRITE_THROUGH)
#define RX_MPU_CACHE_ICACHE_DIS_DCACHE_DIS (RX_MPU_CACHE_ICA_DISABLED | RX_MPU_CACHE_DCA_DISABLED)
#define RX_MPU_CACHE_ALL_DISABLED          (RX_MPU_CACHE_ICA_DISABLED | RX_MPU_CACHE_DCA_DISABLED)
#define RX_MPU_CACHE_ALL_ENABLED           (RX_MPU_CACHE_ICA_ENABLED | RX_MPU_CACHE_DCA_WRITE_BACK)

struct rx_mpu_map {
	/**
	 * Array of MPU partitions.
	 */
	struct z_rx_mpu_partition partitions[RX_MPU_NUM_ENTRIES];
};

/**
 * @brief Initialize the RX MPU hardware.
 * This function initializes the RX MPU hardware
 * and setup the static memory regions at boot.
 */
void rx_mpu_init(void);

#endif /* ZEPHYR_INCLUDE_ARCH_RX_MPU_H */
