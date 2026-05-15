# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CPU_G4MH OR CONFIG_CPU_G4KH)
  set(include_dirs
    Core/Include
    Core/Include/G4-profile
  )

  zephyr_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/${include_dirs})
endif()
