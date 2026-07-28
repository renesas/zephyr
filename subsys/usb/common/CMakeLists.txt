# SPDX-FileCopyrightText: Copyright Nordic Semiconductor ASA
# SPDX-FileCopyrightText: 2026 Microchip, Embedd
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_USBD_VIDEO_CLASS OR CONFIG_USBH_VIDEO_CLASS)
  zephyr_include_directories(.)
  zephyr_sources(uvc.c)
endif()

if(CONFIG_USBD_MSC_CLASS OR CONFIG_USBH_MSC_CLASS)
  zephyr_include_directories(.)
endif()
