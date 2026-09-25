/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _DISPLAY_HANDLE_H_
#define _DISPLAY_HANDLE_H_

#ifdef __cplusplus
extern "C" {
#endif

int display_init(void);

void display_task(void *arg1, void *arg2, void *arg3);

#ifdef __cplusplus
}
#endif

#endif
