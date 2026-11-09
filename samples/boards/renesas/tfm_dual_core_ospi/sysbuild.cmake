# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if("${SB_CONFIG_CM85_NS_BOARD}" STREQUAL "")
  message(FATAL_ERROR "Target ${BOARD}/${BOARD_QUALIFIERS} not supported for this sample. "
    "There is no Cortex-M85 non-secure board selected in Kconfig.sysbuild")
endif()

set(CM85_NS_APP cm85_ns)

ExternalZephyrProject_Add(
  APPLICATION    ${CM85_NS_APP}
  SOURCE_DIR     ${APP_DIR}/cm85
  BOARD          ${SB_CONFIG_CM85_NS_BOARD}
  BOARD_REVISION ${BOARD_REVISION}
)

# ML-DSA-87 key signing both OSPI images. Pass -DOSPI_SIGN_KEY=<pem> for a real
# key; otherwise a throw-away development key is generated in the build
# directory. Both images read it with zephyr_get(OSPI_SIGN_KEY SYSBUILD GLOBAL).
set(OSPI_SIGN_PY ${APP_DIR}/scripts/ospi_sign.py
    --mcuboot-scripts ${ZEPHYR_MCUBOOT_MODULE_DIR}/scripts)
if(NOT OSPI_SIGN_KEY)
  set(dev_key ${CMAKE_BINARY_DIR}/ospi_dev_key_mldsa87.pem)
  if(NOT EXISTS ${dev_key})
    execute_process(COMMAND ${PYTHON_EXECUTABLE} ${OSPI_SIGN_PY} genkey ${dev_key}
                    COMMAND_ERROR_IS_FATAL ANY)
  endif()
  message(WARNING "Using a generated DEVELOPMENT ML-DSA-87 key (${dev_key}); "
                  "pass -DOSPI_SIGN_KEY=<pem> for production")
  set(OSPI_SIGN_KEY ${dev_key} CACHE FILEPATH "ML-DSA-87 key signing the OSPI images" FORCE)
endif()

# One hex file for the whole device: BL2 + TF-M + Cortex-M85 non-secure image
# (MRAM), Cortex-M85 OSPI code and the signed Cortex-M33 image (OSPI)
find_program(SREC_CAT srec_cat REQUIRED)
ExternalProject_Get_Property(${DEFAULT_IMAGE} BINARY_DIR)
set(cm33_binary_dir ${BINARY_DIR})
ExternalProject_Get_Property(${CM85_NS_APP} BINARY_DIR)
set(cm85_binary_dir ${BINARY_DIR})

add_custom_target(merged_hex ALL
  COMMAND ${SREC_CAT} ${cm85_binary_dir}/zephyr_merged.hex -Intel
          ${cm33_binary_dir}/cm33_ospi.hex -Intel
          -o ${CMAKE_BINARY_DIR}/merged.hex -Intel
  DEPENDS ${DEFAULT_IMAGE} ${CM85_NS_APP}
  BYPRODUCTS ${CMAKE_BINARY_DIR}/merged.hex
  COMMENT "Merging all images into merged.hex"
  VERBATIM
)
