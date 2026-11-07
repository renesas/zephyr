# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if("${SB_CONFIG_CM85_NS_BOARD}" STREQUAL "")
  message(FATAL_ERROR "Target ${BOARD}/${BOARD_QUALIFIERS} not supported for this sample. "
    "There is no Cortex-M85 non-secure board selected in Kconfig.sysbuild")
endif()

set(CM85_NS_APP cm85_ns)

ExternalZephyrProject_Add(
  APPLICATION    ${CM85_NS_APP}
  SOURCE_DIR     ${SB_CONFIG_CM85_NS_APP_PATH}
  BOARD          ${SB_CONFIG_CM85_NS_BOARD}
  BOARD_REVISION ${BOARD_REVISION}
)

# The Cortex-M85 non-secure image starts the Cortex-M33.
set_config_bool(${CM85_NS_APP} CONFIG_SOC_RA_ENABLE_START_SECOND_CORE y)

# Flash the Cortex-M85 image first, then the Cortex-M33 image.
sysbuild_add_dependencies(FLASH ${DEFAULT_IMAGE} ${CM85_NS_APP})
