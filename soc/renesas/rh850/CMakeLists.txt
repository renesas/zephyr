# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_include_directories(common)
zephyr_include_directories_ifdef(CONFIG_HAS_RENESAS_RH850_FSP common_fsp)


add_subdirectory(${SOC_SERIES})
