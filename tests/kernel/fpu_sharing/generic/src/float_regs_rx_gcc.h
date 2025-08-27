/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _FLOAT_REGS_RX_GCC_H
#define _FLOAT_REGS_RX_GCC_H

#include <zephyr/toolchain.h>
#include "float_context.h"

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))

static inline void _load_all_float_registers(struct fp_register_set *regs)
{
	__asm__ volatile ("PUSH.L R15\n");

	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc0_l));
	__asm__ volatile ("MOV.L [R15], R15\n");
	__asm__ volatile ("MVTACLO R15, A0\n");
	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc0_h));
	__asm__ volatile ("MOV.L [R15], R15\n");
	__asm__ volatile ("MVTACHI R15, A0\n");

	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc1_l));
	__asm__ volatile ("MOV.L [R15], R15\n");
	__asm__ volatile ("MVTACLO R15, A1\n");
	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc1_h));
	__asm__ volatile ("MOV.L [R15], R15\n");
	__asm__ volatile ("MVTACHI R15, A1\n");

#if defined(CONFIG_DFPU)
	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.dr[0]));
	__asm__ volatile ("DMOV.D    [R15], DR0\n");
	__asm__ volatile ("DMOV.D   8[R15], DR1\n");
	__asm__ volatile ("DMOV.D  16[R15], DR2\n");
	__asm__ volatile ("DMOV.D  24[R15], DR3\n");
	__asm__ volatile ("DMOV.D  32[R15], DR4\n");
	__asm__ volatile ("DMOV.D  40[R15], DR5\n");
	__asm__ volatile ("DMOV.D  48[R15], DR6\n");
	__asm__ volatile ("DMOV.D  56[R15], DR7\n");
	__asm__ volatile ("DMOV.D  64[R15], DR8\n");
	__asm__ volatile ("DMOV.D  72[R15], DR9\n");
	__asm__ volatile ("DMOV.D  80[R15], DR10\n");
	__asm__ volatile ("DMOV.D  88[R15], DR11\n");
	__asm__ volatile ("DMOV.D  96[R15], DR12\n");
	__asm__ volatile ("DMOV.D 104[R15], DR13\n");
	__asm__ volatile ("DMOV.D 112[R15], DR14\n");
	__asm__ volatile ("DMOV.D 120[R15], DR15\n");
#endif

	__asm__ volatile ("POP R15\n");
}

static inline void _store_all_float_registers(struct fp_register_set *regs)
{
	__asm__ volatile ("PUSHM R14-R15\n");

	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc0_l));
	__asm__ volatile ("MVFACLO #0, A0, R14\n");
	__asm__ volatile ("MOV.L R14, [R15]\n");
	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc0_h));
	__asm__ volatile ("MVFACHI #0, A0, R14\n");
	__asm__ volatile ("MOV.L R14, [R15]\n");

	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc1_l));
	__asm__ volatile ("MVFACLO #0, A1, R14\n");
	__asm__ volatile ("MOV.L R14, [R15]\n");
	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.acc1_h));
	__asm__ volatile ("MVFACHI #0, A1, R14\n");
	__asm__ volatile ("MOV.L R14, [R15]\n");

#if defined(CONFIG_DFPU)
	__asm__ volatile ("MOV.L %0, R15\n" :: "r"(&regs->fp_volatile.dr[0]));
	__asm__ volatile ("DMOV.D DR0,     [R15]\n");
	__asm__ volatile ("DMOV.D DR1,    8[R15]\n");
	__asm__ volatile ("DMOV.D DR2,   16[R15]\n");
	__asm__ volatile ("DMOV.D DR3,   24[R15]\n");
	__asm__ volatile ("DMOV.D DR4,   32[R15]\n");
	__asm__ volatile ("DMOV.D DR5,   40[R15]\n");
	__asm__ volatile ("DMOV.D DR6,   48[R15]\n");
	__asm__ volatile ("DMOV.D DR7,   56[R15]\n");
	__asm__ volatile ("DMOV.D DR8,   64[R15]\n");
	__asm__ volatile ("DMOV.D DR9,   72[R15]\n");
	__asm__ volatile ("DMOV.D DR10,  80[R15]\n");
	__asm__ volatile ("DMOV.D DR11,  88[R15]\n");
	__asm__ volatile ("DMOV.D DR12,  96[R15]\n");
	__asm__ volatile ("DMOV.D DR13, 104[R15]\n");
	__asm__ volatile ("DMOV.D DR14, 112[R15]\n");
	__asm__ volatile ("DMOV.D DR15, 120[R15]\n");
#endif

	__asm__ volatile ("POPM R14-R15\n");
}

static inline void _load_then_store_all_float_registers(struct fp_register_set *regs)
{
	_load_all_float_registers(regs);
	_store_all_float_registers(regs);
}

#endif

#endif /* _FLOAT_REGS_RX_GCC_H */
