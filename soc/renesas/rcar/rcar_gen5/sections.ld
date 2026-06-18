/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/devicetree.h>

#ifdef CONFIG_ETH_RENESAS_RCARSOC_TSNES
SECTION_PROLOGUE(.eth_ram,(NOLOAD),SUBALIGN(64))
{
    __eth_ram_start = .;
    . = ALIGN(64);
    KEEP(*(SORT_BY_NAME(.eth_ram*)))
    . = ALIGN(64);
    __eth_ram_end = .;
}
GROUP_LINK_IN(LINKER_DT_NODE_REGION_NAME(DT_NODELABEL(ethuram)))
#endif /* CONFIG_ETH_RENESAS_RCARSOC_TSNES */
