# Copyright (c) 2026 Renesas Electronics Corporation
#
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sent_renesas_rh850)

target_sources(app PRIVATE
    src/main.c
    src/sent_taud_emulator.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
