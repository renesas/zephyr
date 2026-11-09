# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tfm_dual_core_ospi_cm85)

if(NOT CONFIG_BUILD_WITH_TFM)
  message(FATAL_ERROR "The Cortex-M85 image of this sample is a TF-M non-secure image")
endif()

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

# Run ext_code from the external flash (XIP). No need to copy.
zephyr_code_relocate(FILES src/ext_code.c LOCATION EXTFLASH_TEXT NOCOPY)

# But still relocate (copy) the data to RAM
zephyr_code_relocate(FILES src/ext_code.c LOCATION RAM_DATA)

# sram_code instead runs entirely from SRAM after being copied there.
zephyr_code_relocate(FILES src/sram_code.c LOCATION RAM)

# Signature slot linked right after the relocated code in external flash
zephyr_linker_sources(SECTIONS ${CMAKE_CURRENT_SOURCE_DIR}/ospi_sig.ld)

set(OSPI_TEXT_SECTION .extflash_text_reloc)
set(OSPI_SIG_SECTION  .extflash_sig)
set(OSPI_SIGN_PY ${CMAKE_CURRENT_SOURCE_DIR}/../scripts/ospi_sign.py
    --mcuboot-scripts ${ZEPHYR_MCUBOOT_MODULE_DIR}/scripts)

# Key signing the OSPI code and the Cortex-M33 image, set by sysbuild.cmake
zephyr_get(OSPI_SIGN_KEY SYSBUILD GLOBAL)
if(NOT OSPI_SIGN_KEY)
  message(FATAL_ERROR "OSPI_SIGN_KEY is not set: build this sample with --sysbuild")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${OSPI_SIGN_KEY})

# Public key embedded in the MRAM image
execute_process(COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} header ${OSPI_SIGN_KEY}
                        ${PROJECT_BINARY_DIR}/ospi_pubkey.h
                COMMAND_ERROR_IS_FATAL ANY)
target_include_directories(app PRIVATE ${PROJECT_BINARY_DIR})

# zephyr.bin/.hex (the input of the TF-M signing flow) must only contain the
# MRAM image: the OSPI sections live in the OSPI window and would turn the
# binary into a ~2 GB file. They are produced by split_flash below.
set_property(TARGET bintools APPEND PROPERTY elfconvert_flag
             --remove-section=${OSPI_TEXT_SECTION} --remove-section=${OSPI_SIG_SECTION})

# The Non-secure OSPI_B driver reprograms OCTACLK (R_BSP_OctaclkUpdate).
# With secure clocks TF-M makes OCTACKCR Secure and the access faults, so
# hand only OCTACLK (CGFSAR.NONSEC17) to the Non-secure side.
set_property(TARGET zephyr_property_target APPEND PROPERTY TFM_CMAKE_OPTIONS
             -DRENESAS_RA_CGFSAR_NS_OCTACLK=ON)

find_program(SREC_CAT srec_cat REQUIRED)

dt_nodelabel(ext_flash_node NODELABEL ext_flash REQUIRED)
dt_reg_addr(ext_flash_addr PATH ${ext_flash_node})

set(ELF        $<TARGET_FILE:zephyr_final>)
set(SIGNED_ELF ${PROJECT_BINARY_DIR}/zephyr_signed.elf)
set(MERGED_HEX ${PROJECT_BINARY_DIR}/zephyr_merged.hex)

# The NS image is signed by the TF-M (BL2) flow, which produces
# zephyr_ns_signed.* and tfm_merged.hex (BL2 + TF-M S + NS)
set(TFM_MERGED_HEX ${PROJECT_BINARY_DIR}/zephyr/tfm_merged.hex)

add_custom_target(
  split_flash ALL
  DEPENDS zephyr_final
  # Sign the exact bytes the CPU will read from OSPI and patch the
  # signature into its reserved slot
  COMMAND ${CMAKE_OBJCOPY} -O binary -j ${OSPI_TEXT_SECTION} ${ELF}
          ${PROJECT_BINARY_DIR}/ospi_text.bin
  COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} sign ${OSPI_SIGN_KEY}
          ${PROJECT_BINARY_DIR}/ospi_text.bin ${PROJECT_BINARY_DIR}/ospi_sig.bin
  COMMAND ${CMAKE_OBJCOPY} --update-section ${OSPI_SIG_SECTION}=${PROJECT_BINARY_DIR}/ospi_sig.bin
          ${ELF} ${SIGNED_ELF}
  # OSPI data only (intermediate)
  COMMAND ${CMAKE_OBJCOPY} -j ${OSPI_TEXT_SECTION} -j ${OSPI_SIG_SECTION} -O ihex
          ${SIGNED_ELF} ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex
  # Pad to an even length (the image is odd: the signature is 4627 bytes),
  # then byte-swap for octa-mode read order
  COMMAND ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex -Intel
          -fill 0xFF -within ${PROJECT_BINARY_DIR}/zephyr_ospi_raw.hex -Intel -range-padding 2
          -byte_swap 2 -o ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel
  # Flash-ready OSPI binary, starting at offset 0 of the OSPI flash
  COMMAND ${SREC_CAT} ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel
          -offset -${ext_flash_addr} -o ${PROJECT_BINARY_DIR}/zephyr_ospi.signed.bin -Binary
  COMMAND ${SREC_CAT} ${TFM_MERGED_HEX} -Intel
          ${PROJECT_BINARY_DIR}/zephyr_ospi.hex -Intel -o ${MERGED_HEX} -Intel
  COMMENT "Signing and splitting OSPI (${OSPI_TEXT_SECTION}) image for octa-mode read order"
  VERBATIM
)
