# Copyright (c) 2024 EPAM Systems
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_SERIES_RCAR_GEN3 OR CONFIG_SOC_SERIES_RCAR_GEN4)
zephyr_library_sources(pfc_rcar.c)
elseif(CONFIG_SOC_SERIES_RCAR_GEN5)
  zephyr_library_sources(pfc_rcar_x5h.c)
endif()

if(CONFIG_SOC_R8A77951_R7 OR CONFIG_SOC_R8A77951_A57)
  zephyr_library_sources(pfc_r8a77951.c)
elseif(CONFIG_SOC_R8A779F0_R52 OR CONFIG_SOC_R8A779F0_A55)
  zephyr_library_sources(pfc_r8a779f0.c)
elseif(CONFIG_SOC_R8A7800_R52)
  zephyr_library_sources(pfc_r8a7800.c)
endif()

zephyr_library_sources_ifdef(CONFIG_SOC_R8A77961 pfc_r8a77961.c)
