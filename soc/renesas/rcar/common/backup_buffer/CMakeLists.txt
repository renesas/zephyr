# Copyright (c) 2026 Renesas Electronics Corporation
# SPDX-License-Identifier: Apache-2.0

zephyr_sources(rcar_backup_buffer.c)
zephyr_linker_sources(SECTIONS rcar_backup_buffer.ld)
