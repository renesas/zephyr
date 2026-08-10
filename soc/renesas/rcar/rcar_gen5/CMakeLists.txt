# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_R8A78000_R52)
  zephyr_sources(r52/soc.c)
  zephyr_linker_sources(SECTIONS sections.ld)
  zephyr_include_directories(r52)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld CACHE INTERNAL "")
endif()
