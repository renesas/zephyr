# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(tfm_dual_core_ospi)

target_sources(app PRIVATE src/main.c)

# Same key as the public key built into the Cortex-M85 image, set by sysbuild.cmake
zephyr_get(OSPI_SIGN_KEY SYSBUILD GLOBAL)
if(NOT OSPI_SIGN_KEY)
  message(FATAL_ERROR "OSPI_SIGN_KEY is not set: build this sample with --sysbuild")
endif()

find_program(SREC_CAT srec_cat REQUIRED)

dt_nodelabel(cm33_image_node NODELABEL cm33_ospi_partition REQUIRED)
dt_reg_addr(cm33_image_addr PATH ${cm33_image_node})
dt_nodelabel(cm33_manifest_node NODELABEL cm33_ospi_manifest_partition REQUIRED)
dt_reg_addr(cm33_manifest_addr PATH ${cm33_manifest_node})

set(OSPI_SIGN_PY ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ospi_sign.py
    --mcuboot-scripts ${ZEPHYR_MCUBOOT_MODULE_DIR}/scripts)
set(CM33_BIN      ${ZEPHYR_BINARY_DIR}/${KERNEL_BIN_NAME})
set(CM33_MANIFEST ${CMAKE_BINARY_DIR}/cm33_manifest.bin)
set(CM33_RAW_HEX  ${CMAKE_BINARY_DIR}/cm33_ospi_raw.hex)
set(CM33_OSPI_HEX ${CMAKE_BINARY_DIR}/cm33_ospi.hex)

add_custom_target(
  cm33_ospi_image ALL
  DEPENDS zephyr_final
  # Size and signature of the exact bytes the Cortex-M33 fetches from OSPI
  COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} manifest ${OSPI_SIGN_KEY} ${CM33_BIN}
          ${CM33_MANIFEST}
  COMMAND ${SREC_CAT} ${CM33_BIN} -Binary -offset ${cm33_image_addr}
          ${CM33_MANIFEST} -Binary -offset ${cm33_manifest_addr}
          -o ${CM33_RAW_HEX} -Intel
  # Pad each range to an even length, then byte-swap for octa-mode read order
  COMMAND ${SREC_CAT} ${CM33_RAW_HEX} -Intel
          -fill 0xFF -within ${CM33_RAW_HEX} -Intel -range-padding 2
          -byte_swap 2 -o ${CM33_OSPI_HEX} -Intel
  BYPRODUCTS ${CM33_MANIFEST} ${CM33_RAW_HEX} ${CM33_OSPI_HEX}
  COMMENT "Signing Cortex-M33 OSPI image"
  VERBATIM
)
