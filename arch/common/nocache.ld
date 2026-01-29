/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2019 Intel Corporation
 * Copyright (c) 2025 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Copied from linker.ld */

/* Non-cached region of RAM */
SECTION_DATA_PROLOGUE(_NOCACHE_SECTION_NAME,(NOLOAD),)
{
#if defined(CONFIG_MMU)
	MMU_ALIGN;
#else
	MPU_ALIGN(_nocache_ram_size);
#endif
	PLACE_SYMBOL_HERE(_nocache_ram_start);
	PLACE_SYMBOL_HERE(_nocache_noload_ram_start);
	*(.nocache)
	*(".nocache.*")

#include <snippets-nocache-section.ld>

	PLACE_SYMBOL_HERE(_nocache_noload_ram_end);
} GROUP_DATA_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
#if defined(CONFIG_RX)
__nocache_noload_ram_size = _nocache_noload_ram_end - _nocache_noload_ram_start;
PROVIDE(_nocache_noload_ram_size = __nocache_noload_ram_size);
#else
_nocache_noload_ram_size = _nocache_noload_ram_end - _nocache_noload_ram_start;
#endif

/* Non-cached loadable region of RAM and ROM */
SECTION_DATA_PROLOGUE(_NOCACHE_LOAD_SECTION_NAME,,)
{
	PLACE_SYMBOL_HERE(_nocache_load_ram_start);
	*(.nocache_load)
	*(".nocache_load.*")

#if defined(CONFIG_MMU)
	MMU_ALIGN;
#else
	MPU_ALIGN(_nocache_ram_size);
#endif
	PLACE_SYMBOL_HERE(_nocache_load_ram_end);
	PLACE_SYMBOL_HERE(_nocache_ram_end);
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)
#if defined(CONFIG_RX)
__nocache_load_ram_size = _nocache_load_ram_end - _nocache_load_ram_start;
__nocache_load_rom_start = LOADADDR(_NOCACHE_LOAD_SECTION_NAME);
__nocache_ram_size = _nocache_ram_end - _nocache_ram_start;
PROVIDE(_nocache_load_ram_size = __nocache_load_ram_size);
PROVIDE(_nocache_load_rom_start = __nocache_load_rom_start);
PROVIDE(_nocache_ram_size = __nocache_ram_size);
#else
_nocache_load_ram_size = _nocache_load_ram_end - _nocache_load_ram_start;
_nocache_load_rom_start = LOADADDR(_NOCACHE_LOAD_SECTION_NAME);
_nocache_ram_size = _nocache_ram_end - _nocache_ram_start;
#endif
