/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>

#if defined(CONFIG_ETH_RENESAS_RCARSOC_TSNES) || defined(CONFIG_ETH_RENESAS_RSWITCH3_GWCA)
SECTION_PROLOGUE(.eth_ram,(NOLOAD),SUBALIGN(64))
{
    __eth_ram_start = .;
    . = ALIGN(64);
    KEEP(*(SORT_BY_NAME(.eth_ram*)))
    . = ALIGN(64);
    __eth_ram_end = .;
}
GROUP_LINK_IN(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(ethuram)))
#endif /* CONFIG_ETH_RENESAS_RCARSOC_TSNES || CONFIG_ETH_RENESAS_RSWITCH3_GWCA */

#ifdef CONFIG_OPENAMP_RSC_TABLE

#if !defined(CONFIG_OPENAMP_COPY_RSC_TABLE) && DT_HAS_CHOSEN(zephyr_ipc_rsc_table)
#if DT_NODE_HAS_PROP(DT_CHOSEN(zephyr_ipc_rsc_table), zephyr_memory_region)
#define RCAR_RESOURCE_TABLE_REGION LINKER_DT_NODE_REGION_NAME_TOKEN(DT_CHOSEN(zephyr_ipc_rsc_table))
#endif
#endif

#ifndef RCAR_RESOURCE_TABLE_REGION
#define RCAR_RESOURCE_TABLE_REGION ROMABLE_REGION
#endif

SECTION_PROLOGUE(.resource_table,, SUBALIGN(4))
{
	KEEP(*(.resource_table*))
} GROUP_LINK_IN(RCAR_RESOURCE_TABLE_REGION)

#undef RCAR_RESOURCE_TABLE_REGION

#endif /* CONFIG_OPENAMP_RSC_TABLE */
