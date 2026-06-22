/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * Linker script for the RX platforms.
 */

#include <zephyr/linker/sections.h>
#include <zephyr/devicetree.h>

#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

#define EXTFLASH_NODE	DT_INST(0, renesas_rx_xspi_nor)
#define EXTFLASH_ADDR	0x50000000
#define EXTFLASH_SIZE	DT_PROP_OR(EXTFLASH_NODE, size_in_bytes, \
				   DT_PROP(EXTFLASH_NODE, size) / 8)

MEMORY
{
	EXTFLASH (rx) : ORIGIN = EXTFLASH_ADDR, LENGTH = EXTFLASH_SIZE
}

#include <zephyr/arch/rx/linker.ld>
