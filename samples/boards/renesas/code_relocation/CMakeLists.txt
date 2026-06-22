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
