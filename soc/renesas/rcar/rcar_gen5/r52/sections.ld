/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>

#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_dtcm), okay)
GROUP_START(DTCM)

	SECTION_PROLOGUE(_DTCM_BSS_SECTION_NAME, (NOLOAD),SUBALIGN(4))
	{
		__dtcm_start = .;
		__dtcm_bss_start = .;
		*(.dtcm_bss)
		*(".dtcm_bss.*")
		__dtcm_bss_end = .;
	} GROUP_LINK_IN(DTCM)

	SECTION_PROLOGUE(_DTCM_NOINIT_SECTION_NAME, (NOLOAD),SUBALIGN(4))
	{
		__dtcm_noinit_start = .;
		*(.dtcm_noinit)
		*(".dtcm_noinit.*")
		__dtcm_noinit_end = .;
	} GROUP_LINK_IN(DTCM)

	SECTION_PROLOGUE(_DTCM_DATA_SECTION_NAME,,SUBALIGN(4))
	{
		__dtcm_data_start = .;
		*(.dtcm_data)
		*(".dtcm_data.*")

/* Located in generated directory. This file is populated by the
 * zephyr_linker_sources() Cmake function. */
#include <snippets-dtcm-section.ld>

		__dtcm_data_end = .;
	} GROUP_LINK_IN(DTCM AT> ROMABLE_REGION)

	__dtcm_end = .;

	__dtcm_data_load_start = LOADADDR(_DTCM_DATA_SECTION_NAME);

GROUP_END(DTCM)
#endif
