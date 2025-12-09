/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Copied from linker.ld */

SECTION_DATA_PROLOGUE(.ramfunc,,)
{
        PLACE_SYMBOL_HERE(__ramfunc_region_start);
        MPU_ALIGN(__ramfunc_size);
        PLACE_SYMBOL_HERE(__ramfunc_start);
        *(.ramfunc)
        *(".ramfunc.*")

#include <snippets-ramfunc-section.ld>

        MPU_ALIGN(__ramfunc_size);
        PLACE_SYMBOL_HERE(__ramfunc_end);
} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

#if defined(CONFIG_RX)
___ramfunc_size = ___ramfunc_end - ___ramfunc_start;
___ramfunc_load_start = LOADADDR(.ramfunc);
PROVIDE(__ramfunc_size = ___ramfunc_size);
PROVIDE(__ramfunc_load_start = ___ramfunc_load_start);
#else
__ramfunc_size = __ramfunc_end - __ramfunc_start;
__ramfunc_load_start = LOADADDR(.ramfunc);
#endif
