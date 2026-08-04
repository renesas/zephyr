# SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/usb/class/usbh_hid.h)

zephyr_library_sources(
  usbh_hid.c
  hid_report.c
)

