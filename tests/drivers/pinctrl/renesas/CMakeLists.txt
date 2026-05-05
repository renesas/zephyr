# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(pinctrl_renesas)

target_sources(app PRIVATE ../common/test_device.c)
if(CONFIG_PINCTRL_RCAR_PFC)
  target_sources(app PRIVATE src/main_rcar.c)
endif()
