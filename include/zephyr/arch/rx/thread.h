/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RX_THREAD_H_
#define ZEPHYR_INCLUDE_ARCH_RX_THREAD_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct _callee_saved {
	/* General purpose callee-saved registers */
};

typedef struct _callee_saved _callee_saved_t;

/* Forward declaration; full definition in <zephyr/arch/rx/mpu.h>. */
struct rx_mpu_map;

struct _thread_arch {
#ifdef CONFIG_USERSPACE
	/* Active MPU map (points at the thread's memory-domain map). */
	struct rx_mpu_map *mpu_map;
#else
	/* empty */
#endif
};

typedef struct _thread_arch _thread_arch_t;

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RX_THREAD_H_ */
