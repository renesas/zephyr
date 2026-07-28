# SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(test_usbh_msc)

if(${BOARD} STREQUAL "native_sim")
  include(${ZEPHYR_BASE}/samples/subsys/usb/common/common.cmake)
endif()

target_include_directories(app PRIVATE ${ZEPHYR_BASE}/tests/subsys/usb/host/common)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/usb/host)

target_sources(app PRIVATE src/msc.c)

if(${BOARD} STREQUAL "native_sim")
    target_sources(app PRIVATE src/ram_disk.c)
endif()
