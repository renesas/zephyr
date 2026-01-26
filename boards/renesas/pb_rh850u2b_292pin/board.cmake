# Copyright (c) 2025 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

board_runner_args(rfp "-device U2x" "-tool E2" "-interface=CSI" "-erase")

include(${ZEPHYR_BASE}/boards/common/rfp.board.cmake)
