# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_RCAR_IRONHIDE_X5H_R8A7800_R52)
  board_runner_args(openocd "--file-type=elf")
  include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
endif()
