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
 * Renesas RA OSPI_B: the external flash is mapped in the XIP region at
 * 0x9000_0000 (the "ranges" of the flash controller node). The last 4 KiB
 * hold the driver's auto-calibration pattern and must not be used for code.
 */

#define EXTFLASH_NODE	DT_NODELABEL(ospi0_ext_flash)
#define EXTFLASH_ADDR	0x90000000
#define EXTFLASH_SIZE	(DT_REG_SIZE(EXTFLASH_NODE) - 4096)

MEMORY
{
	EXTFLASH (rx) : ORIGIN = EXTFLASH_ADDR, LENGTH = EXTFLASH_SIZE
}

#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>
