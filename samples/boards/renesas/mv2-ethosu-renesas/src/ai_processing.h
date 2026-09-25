/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _AI_PROCESSING_H_
#define _AI_PROCESSING_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Loads the ExecuTorch program/method once. Must be called before ai_task. */
int ai_init(void);

void ai_task(void *arg1, void *arg2, void *arg3);

#ifdef __cplusplus
}
#endif

#endif
