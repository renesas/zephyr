/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_ARCH_RX_INLINES_H_
#define ZEPHYR_INCLUDE_ARCH_RX_INLINES_H_

#ifndef _ASMLANGUAGE
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct arch_esf {
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING) && defined(CONFIG_DFPU))
	uint32_t dpsw;
	uint32_t dcmr;
	uint32_t decnt;
	uint32_t dr0_l;
	uint32_t dr0_h;
	uint32_t dr1_l;
	uint32_t dr1_h;
	uint32_t dr2_l;
	uint32_t dr2_h;
	uint32_t dr3_l;
	uint32_t dr3_h;
	uint32_t dr4_l;
	uint32_t dr4_h;
	uint32_t dr5_l;
	uint32_t dr5_h;
	uint32_t dr6_l;
	uint32_t dr6_h;
	uint32_t dr7_l;
	uint32_t dr7_h;
	uint32_t dr8_l;
	uint32_t dr8_h;
	uint32_t dr9_l;
	uint32_t dr9_h;
	uint32_t dr10_l;
	uint32_t dr10_h;
	uint32_t dr11_l;
	uint32_t dr11_h;
	uint32_t dr12_l;
	uint32_t dr12_h;
	uint32_t dr13_l;
	uint32_t dr13_h;
	uint32_t dr14_l;
	uint32_t dr14_h;
	uint32_t dr15_l;
	uint32_t dr15_h;
#endif
#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
	uint32_t acc0_l;
	uint32_t acc0_h;
	uint32_t acc0_g;
	uint32_t acc1_l;
	uint32_t acc1_h;
	uint32_t acc1_g;
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
	uint32_t fpsw;
#endif
#elif defined(CONFIG_CPU_RXV1)
	uint32_t acc_l;
	uint32_t acc_h;
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
	uint32_t fpsw;
#endif
#endif
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t r10;
	uint32_t r11;
	uint32_t r12;
	uint32_t r13;
	uint32_t r14;
	uint32_t r15;
	uint32_t entry_point;
	uint32_t psw;
};

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_RX_INLINES_H_ */
