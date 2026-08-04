/*
 * SPDX-FileCopyrightText: Copyright 2026 Renesas, Embedd
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HID_KEYBOARD_H_INCLUDED
#define HID_KEYBOARD_H_INCLUDED

#include <stdint.h>

int hid_keyboard_register(void);
uint8_t hid_keyboard_get_report_value(void);

#endif /* HID_KEYBOARD_H_INCLUDED */
