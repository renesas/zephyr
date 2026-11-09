/*
 * Copyright (c) 2022 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * Linker script for the Cortex-M platforms.
 */

#include <zephyr/linker/sections.h>
#include <zephyr/devicetree.h>

#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

/*
 * Renesas RA OSPI_B: the code relocated to EXTFLASH_TEXT is linked into the
 * Cortex-M85 partition of the memory-mapped OSPI flash (ospi_layout.dtsi).
 * The rest of the flash holds the Cortex-M33 image and the auto-calibration
 * pattern.
 */

#define EXTFLASH_NODE	DT_NODELABEL(cm85_ospi_partition)

MEMORY
{
	EXTFLASH (rx) : ORIGIN = DT_REG_ADDR(EXTFLASH_NODE), LENGTH = DT_REG_SIZE(EXTFLASH_NODE)
}

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>
