# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/pwm3p.h)

zephyr_library()

# zephyr-keep-sorted-start
zephyr_library_sources_ifdef(CONFIG_PWM3P_RENESAS_RH850_TAUD3P pwm3p_renesas_rh850_taud3p.c)
# zephyr-keep-sorted-stop
