# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_R8A78000_R52)
  zephyr_sources(r52/soc.c)
  zephyr_sources_ifdef(CONFIG_ARM_MPU r52/mpu_regions.c)
  zephyr_linker_sources(SECTIONS sections.ld)
  zephyr_include_directories(r52)
  set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/arm/cortex_a_r/scripts/linker.ld CACHE INTERNAL "")
  zephyr_linker_sources(SECTIONS r52/sections.ld)
  zephyr_linker_sources(ITCM_SECTION r52/itcm.ld)
endif()

set_property(GLOBAL APPEND PROPERTY extra_post_build_commands
  COMMAND ${CMAKE_OBJCOPY} -O srec --srec-forceS3
  ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.elf
  ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.srec
)

set_property(GLOBAL APPEND PROPERTY extra_post_build_byproducts
  ${PROJECT_BINARY_DIR}/${CONFIG_KERNEL_BIN_NAME}.srec
)

zephyr_ld_options(-Wl,--no-warn-rwx-segments)
