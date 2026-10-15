# Copyright 2025 NXP
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/psi5/psi5.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_PSI5_NXP_S32 psi5_nxp_s32.c)
zephyr_library_sources_ifdef(CONFIG_PSI5_RENESAS_RH850 psi5_renesas_rh850.c)
