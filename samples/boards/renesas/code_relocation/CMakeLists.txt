# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(code_relocation_nocopy)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Run ext_code from PSRAM (copied from flash at startup) on RX, or XIP on others.
if (CONFIG_BOOTLOADER_MCUBOOT)
zephyr_code_relocate(FILES src/ext_code.c LOCATION PSRAM_SLOT0_TEXT)
endif()

# But still relocate (copy) the data to RAM
zephyr_code_relocate(FILES src/ext_code.c LOCATION RAM_DATA)

# sram_code instead runs entirely from SRAM after being copied there.
zephyr_code_relocate(FILES src/sram_code.c LOCATION RAM)

# Relocate the RX context switch code to SRAM
zephyr_code_relocate(FILES ${ZEPHYR_BASE}/arch/rx/core/switch.S LOCATION RAM)

# Relocate the FSP group IRQ code to SRAM. NOKEEP drops the unused
# group_dmac_irq_isr, which references a callback list this sample does not define.
zephyr_code_relocate(FILES ${ZEPHYR_HAL_RENESAS_MODULE_DIR}/drivers/rx/fsp/src/bsp/mcu/all/bsp_group_irq.c
  LOCATION RAM NOKEEP)
