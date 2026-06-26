/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Linker command/script file
 *
 * Generic Linker script for the riscv platform
 */

#include <zephyr/devicetree.h>

#include <zephyr/linker/sections.h>
#include <zephyr/linker/devicetree_regions.h>

#include <zephyr/linker/linker-defs.h>
#include <zephyr/linker/linker-tool.h>

#ifdef CONFIG_XIP
#define ROMABLE_REGION	ROM
#else
#define ROMABLE_REGION	RAM
#endif
#define RAMABLE_REGION	RAM

#if defined(CONFIG_ROM_END_OFFSET)
#define ROM_END_OFFSET CONFIG_ROM_END_OFFSET
#else
#define ROM_END_OFFSET 0
#endif

#if defined(CONFIG_FLASH_LOAD_OFFSET)
#define FLASH_LOAD_OFFSET CONFIG_FLASH_LOAD_OFFSET
#else
#define FLASH_LOAD_OFFSET 0
#endif

#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_flash), okay)
#define ROM_START (CONFIG_FLASH_BASE_ADDRESS + FLASH_LOAD_OFFSET)
#ifndef ROM_SIZE
#if CONFIG_FLASH_LOAD_SIZE > 0
#define ROM_SIZE (CONFIG_FLASH_LOAD_SIZE - ROM_END_OFFSET)
#else
#define ROM_SIZE (DT_REG_SIZE(DT_CHOSEN(zephyr_flash)) - ROM_END_OFFSET)
#endif
#endif
#endif

#define RAM_START (CONFIG_SRAM_BASE_ADDRESS)
#define RAM_SIZE  (KB(CONFIG_SRAM_SIZE))
#ifdef CONFIG_RX_MPU_ENABLE
	#define MPU_ALIGN(region_size) . = ALIGN(CONFIG_RX_MPU_MIN_ALIGN_REGION_SIZE)
	#define MPU_MIN_SIZE_ALIGN . = ALIGN(CONFIG_RX_MPU_MIN_ALIGN_REGION_SIZE)
#else
	#define MPU_MIN_SIZE_ALIGN
	#define MPU_ALIGN(region_size) . = ALIGN(4)
#endif

MEMORY
{
	ROM (rx) : ORIGIN = ROM_START, LENGTH = ROM_SIZE
	RAM (rwx): ORIGIN = RAM_START, LENGTH = RAM_SIZE

	LINKER_DT_REGIONS()
	/* Used by and documented in include/linker/intlist.ld */
	IDT_LIST (wx) : ORIGIN = 0xFFFFF780, LENGTH = 2K
#if !defined(CONFIG_BOOTLOADER_MCUBOOT)
	VECTORS  (rx) : ORIGIN = 0xFFFFFF80, LENGTH = 0x80   /* exvectors */
	RESETVEC (rx) : ORIGIN = 0xFFFFFFFC, LENGTH = 0x4    /* fvectors */

#endif
}

ENTRY(CONFIG_KERNEL_ENTRY)

SECTIONS
{

#include <zephyr/linker/rel-sections.ld>
#include <zephyr/linker/llext-sections.ld>

	GROUP_START(ROMABLE_REGION)
	. = ROM_START;   /* for kernel logging */
	PLACE_SYMBOL_HERE(__rom_region_start);

#if !defined(CONFIG_BOOTLOADER_MCUBOOT)
	#if CONFIG_HAS_EXCEPT_VECTOR_TABLE
		.exvectors 0xFFFFFF80 : AT(0xFFFFFF80)
		{
			PLACE_SYMBOL_HERE(__exvectors_start);
			KEEP(*(.exvectors))
			PLACE_SYMBOL_HERE(__exvectors_end);
		} > VECTORS

		.fvectors 0xFFFFFFFC : AT(0xFFFFFFFC)
		{
			KEEP(*(.fvectors))
		} > RESETVEC
	#else
		.fvectors 0xFFFFFF80: AT(0xFFFFFF80)
		{
			KEEP(*(.fvectors))
		} > VECTORS
	#endif
#else
	SECTION_PROLOGUE(.exvectors, ROM_START,)
	{
/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-rom-start.ld>
		PLACE_SYMBOL_HERE(__exvectors_start);
		KEEP(*(.exvectors))
		PLACE_SYMBOL_HERE(__exvectors_end);
	} GROUP_LINK_IN(ROMABLE_REGION)
#endif

	SECTION_PROLOGUE(.startup_entry,,SUBALIGN(4))
	{
		KEEP(*(.startup_entry))
		. = ALIGN(4);
		PLACE_SYMBOL_HERE(__startup_entry_end);
	} GROUP_LINK_IN(ROMABLE_REGION)

#ifdef CONFIG_CODE_DATA_RELOCATION
#include <linker_relocate.ld>
#endif /* CONFIG_CODE_DATA_RELOCATION */

	SECTION_PROLOGUE(_TEXT_SECTION_NAME,,SUBALIGN(4))
	{
		. = ALIGN(4);
		PLACE_SYMBOL_HERE(__image_text_start);
		*(.text)
		*(.text.*)
		*(P)
		. = ALIGN(4);
		etext = .;
#include <zephyr/linker/kobject-text.ld>
	} GROUP_LINK_IN(ROMABLE_REGION)

	PLACE_SYMBOL_HERE(__image_text_end);

	SECTION_PROLOGUE(.rvectors,,)
	{
		_rvectors_start = .;
		KEEP(*(.rvectors))
		_rvectors_end = .;
	} GROUP_LINK_IN(ROMABLE_REGION)

	PLACE_SYMBOL_HERE(__rodata_region_start);

	#include <zephyr/linker/common-rom.ld>
	/* Located in generated directory. This file is populated by calling
 	* zephyr_linker_sources(ROM_SECTIONS ...). Useful for grouping iterable RO structs.
 	*/
#include <snippets-rom-sections.ld>
#include <zephyr/linker/thread-local-storage.ld>

    	SECTION_PROLOGUE(_RODATA_SECTION_NAME,,)
	{
		. = ALIGN(8);
		*(.rodata)
		*(.rodata.*)
		*(.gnu.linkonce.r)
		*(.gnu.linkonce.r.*)
		*(C_1)
		*(C_2)
		*(C)

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-rodata.ld>

	} GROUP_LINK_IN(ROMABLE_REGION)

/* Located in generated directory. This file is populated by calling
 * zephyr_linker_sources(ROM_SECTIONS ...). Useful for grouping iterable RO structs.
 */
#include <snippets-rom-sections.ld>


	SECTION_PROLOGUE(init,,)
	{
		KEEP(*(.init))
		__preinit_array_start = .;
		KEEP(*(.preinit_array))
		__preinit_array_end = .;
		__init_array_start = (. + 3) & ~ 3;
		KEEP(*(.init_array))
		KEEP(*(SORT(.init_array.*)))
		__init_array_end = .;
		__fini_array_start = .;
		KEEP(*(.fini_array))
		KEEP(*(SORT(.fini_array.*)))
		__fini_array_end = .;
	} GROUP_LINK_IN(ROMABLE_REGION)

	SECTION_PROLOGUE(fini,,)
	{
		KEEP(*(.fini))
	} GROUP_LINK_IN(ROMABLE_REGION)

	SECTION_PROLOGUE(got,,)
	{
		*(.got)
		*(.got.plt)
	} GROUP_LINK_IN(ROMABLE_REGION)

#include <zephyr/linker/cplusplus-rom.ld>

#if DT_PROP(DT_NODELABEL(code_flash), programming_enable)
	SECTION_PROLOGUE(.pfram,ALIGN(4),)
	{
		_PFRAM_start = .;
		. += _RPFRAM_end - _RPFRAM_start;
		_PFRAM_end = .;
	} > ROMABLE_REGION
#endif

	/* TODO: is this section necessary? There is a similar section
	 * (_CTOR_SECTION_NAME) in common-rom.ld. This seems to be for
	 * C++ Constructors/Destructors? */
	SECTION_PROLOGUE(tors,,)
	{
		__CTOR_LIST__ = .;
		. = ALIGN(2);
		__ctors = .;
		*(.ctors)
		__ctors_end = .;
		__CTOR_END__ = .;
		__DTOR_LIST__ = .;
		__dtors = .;
		*(.dtors)
		__dtors_end = .;
		__DTOR_END__ = .;
		. = ALIGN(2);
	} GROUP_LINK_IN(ROMABLE_REGION)

	PLACE_SYMBOL_HERE(__rodata_region_end);

	MPU_ALIGN(__rodata_region_end - ADDR(__rodata_region_start));

	GROUP_END(ROMABLE_REGION)

	GROUP_START(RAMABLE_REGION)

	. = RAM_START;
	PLACE_SYMBOL_HERE(_image_ram_start);

#include <app_data_alignment.ld>

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-ram-sections.ld>

#if defined(CONFIG_USERSPACE)
#define APP_SHARED_ALIGN MPU_MIN_SIZE_ALIGN
#define SMEM_PARTITION_ALIGN MPU_ALIGN

#include <app_smem.ld>
	_app_smem_size = _app_smem_end - _app_smem_start;
	_app_smem_rom_start = LOADADDR(_APP_SMEM_SECTION_NAME);
#endif /* CONFIG_USERSPACE */

	#if CONFIG_SRAM_BASE_ADDRESS == 0
		/* RX memory starts at address 0 which can be confused with NULL. To prevent this, block
		 * the first memory page (16 Bytes).
		 */
		SECTION_DATA_PROLOGUE(.null_blocker,,)
		{
			. = 0x10;
		} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, RAMABLE_REGION)
	#endif

	SECTION_DATA_PROLOGUE(_DATA_SECTION_NAME,,)
	{
		PLACE_SYMBOL_HERE(__data_region_start);
		PLACE_SYMBOL_HERE(__data_start);
		*(.data)
		*(.data.*)
		*(D)
		*(D_1)
		*(D_2)
		*(.gnu.linkonce.*)

#include <snippets-rwdata.ld>

#ifdef CONFIG_CODE_DATA_RELOCATION
#include <linker_sram_data_relocate.ld>
#endif
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	___data_region_load_start = LOADADDR(_DATA_SECTION_NAME);
	PROVIDE(__data_region_load_start = ___data_region_load_start);

	/* the sections defined in common-ram.ld have to be initialized on
	 * reset as well, to place them before _edata */
#include <zephyr/linker/common-ram.ld>
#include <zephyr/linker/kobject-data.ld>
#include <zephyr/linker/cplusplus-ram.ld>

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-data-sections.ld>

	PLACE_SYMBOL_HERE(__data_region_end);

#if DT_PROP(DT_NODELABEL(code_flash), programming_enable)

	.rpfram ALIGN(4): AT(_PFRAM_start)
	{
		_RPFRAM_start = .;
		*(PFRAM)
		. = ALIGN(4);
		_RPFRAM_end = .;
	} > RAMABLE_REGION

#endif

	/* Sections generated from 'zephyr,memory-region' nodes */
	LINKER_DT_SECTIONS()

	SECTION_PROLOGUE(_BSS_SECTION_NAME,,)
	{
		PLACE_SYMBOL_HERE(_bss);
		PLACE_SYMBOL_HERE(__bss_start);
		*(.bss)
		*(.bss.**)
		*(COMMON)
		*(B)
		*(B_1)
		*(B_2)

#ifdef CONFIG_CODE_DATA_RELOCATION
#include <linker_sram_bss_relocate.ld>
#endif

		PLACE_SYMBOL_HERE(_ebss);
		PLACE_SYMBOL_HERE(__bss_end);
		. = ALIGN(128);
	} GROUP_LINK_IN(RAMABLE_REGION)

	_ebss = . ;

#include <zephyr/linker/common-noinit.ld>

	_image_ram_end = .;
	_end = .;
	PROVIDE(__end = _end);

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function.
 */
#include <snippets-sections.ld>

#include <zephyr/linker/ram-end.ld>

	GROUP_END(RAMABLE_REGION)

#include <zephyr/linker/debug-sections.ld>

}
