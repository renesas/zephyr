# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(common)
add_subdirectory_ifdef(CONFIG_RENESAS_PIC common/pic)


add_subdirectory(${SOC_SERIES})
