# Copyright (c) 2025-2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_SOC_R7KA8M2JFLCAC_CM85)
  board_runner_args(jlink "--device=R7KA8M2JF_CPU0" "--reset-after-load")
else()
  board_runner_args(jlink "--device=R7KA8M2JF_CPU1")
endif()

board_runner_args(pyocd "--target=R7KA8M2JF")

if(CONFIG_BUILD_WITH_TFM)
  # Program the TF-M partition data file setting the TrustZone boundaries,
  # then the image to flash (the merged TF-M and Zephyr image by default).
  if(CONFIG_TFM_BL2)
    set(_rfp_rpd_file bl2.rpd)
  else()
    set(_rfp_rpd_file tfm_s.rpd)
  endif()

  board_runner_args(rfp "--device=RA" "--tool=jlink" "--interface=swd"
    "--erase-chip"
    "--rpd-file=${PROJECT_BINARY_DIR}/${_rfp_rpd_file}"
  )
else()
  board_runner_args(rfp "--device=RA" "--tool=jlink" "--interface=swd")
endif()

if(CONFIG_TFM_FLASH_MERGED_BINARY)
  set_property(TARGET runners_yaml_props_target PROPERTY hex_file tfm_merged.hex)
endif()

if(CONFIG_BUILD_WITH_TFM)
  # The TF-M partition data (.rpd) can only be programmed by RFP, so make it
  # the default runner. The first board_set_flasher_ifnset() call wins, hence
  # this must come before the other runner includes.
  board_set_flasher_ifnset(rfp)
endif()

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/rfp.board.cmake)
