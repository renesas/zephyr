/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/arm64/arm_mmu.h>
#include <zephyr/devicetree.h>

static const struct arm_mmu_region mmu_regions[] = {
	MMU_REGION_FLAT_ENTRY("GICD", DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 0),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 0),
			      MT_DEVICE_nGnRnE | MT_RW | MT_DEFAULT_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("GICR", DT_REG_ADDR_BY_IDX(DT_INST(0, arm_gic), 1),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, arm_gic), 1),
			      MT_DEVICE_nGnRnE | MT_RW | MT_DEFAULT_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("GPIO", DT_REG_ADDR_BY_IDX(DT_INST(0, renesas_rz_gpio_common), 0),
			      DT_REG_SIZE_BY_IDX(DT_INST(0, renesas_rz_gpio_common), 0),
			      MT_DEVICE_nGnRnE | MT_RW | MT_DEFAULT_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("SYC", 0x11000000, 0x10000,
			      MT_DEVICE_nGnRnE | MT_RW | MT_DEFAULT_SECURE_STATE),

	MMU_REGION_FLAT_ENTRY("CPG", 0x11010000, 0x10000,
			      MT_DEVICE_nGnRnE | MT_RW | MT_DEFAULT_SECURE_STATE),
};

const struct arm_mmu_config mmu_config = {
	.num_regions = ARRAY_SIZE(mmu_regions),
	.mmu_regions = mmu_regions,
};
