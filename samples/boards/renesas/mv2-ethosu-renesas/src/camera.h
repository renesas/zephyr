/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _CAMERA_HANDLE_H_
#define _CAMERA_HANDLE_H_

int camera_init(void);

void camera_task(void *arg1, void *arg2, void *arg3);

void camera_ai_preprocess_task(void *arg1, void *arg2, void *arg3);

#endif
