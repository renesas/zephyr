/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Fatal fault handling
 *
 * This module implements the routines necessary for handling fatal faults on
 * RX CPUs.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <kernel_arch_data.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

#ifdef CONFIG_EXCEPTION_DEBUG
static void dump_rx_esf(const struct arch_esf *esf)
{
#if defined(CONFIG_CPU_RXV1)
	EXCEPTION_DUMP(" ACC_L: 0x%08x  ACC_H:  0x%08x", esf->acc_l, esf->acc_h);
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
	EXCEPTION_DUMP(" FPSW:  0x%08x", esf->fpsw);
#endif

#elif defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E)
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING) && defined(CONFIG_DFPU))
	EXCEPTION_DUMP(" DR0_L:  0x%08x  DR0_H:  0x%08x", esf->dr0_l,  esf->dr0_h);
	EXCEPTION_DUMP(" DR1_L:  0x%08x  DR1_H:  0x%08x", esf->dr1_l,  esf->dr1_h);
	EXCEPTION_DUMP(" DR2_L:  0x%08x  DR2_H:  0x%08x", esf->dr2_l,  esf->dr2_h);
	EXCEPTION_DUMP(" DR3_L:  0x%08x  DR3_H:  0x%08x", esf->dr3_l,  esf->dr3_h);
	EXCEPTION_DUMP(" DR4_L:  0x%08x  DR4_H:  0x%08x", esf->dr4_l,  esf->dr4_h);
	EXCEPTION_DUMP(" DR5_L:  0x%08x  DR5_H:  0x%08x", esf->dr5_l,  esf->dr5_h);
	EXCEPTION_DUMP(" DR6_L:  0x%08x  DR6_H:  0x%08x", esf->dr6_l,  esf->dr6_h);
	EXCEPTION_DUMP(" DR7_L:  0x%08x  DR7_H:  0x%08x", esf->dr7_l,  esf->dr7_h);
	EXCEPTION_DUMP(" DR8_L:  0x%08x  DR8_H:  0x%08x", esf->dr8_l,  esf->dr8_h);
	EXCEPTION_DUMP(" DR9_L:  0x%08x  DR9_H:  0x%08x", esf->dr9_l,  esf->dr9_h);
	EXCEPTION_DUMP(" DR10_L: 0x%08x  DR10_H: 0x%08x", esf->dr10_l, esf->dr10_h);
	EXCEPTION_DUMP(" DR11_L: 0x%08x  DR11_H: 0x%08x", esf->dr11_l, esf->dr11_h);
	EXCEPTION_DUMP(" DR12_L: 0x%08x  DR12_H: 0x%08x", esf->dr12_l, esf->dr12_h);
	EXCEPTION_DUMP(" DR13_L: 0x%08x  DR13_H: 0x%08x", esf->dr13_l, esf->dr13_h);
	EXCEPTION_DUMP(" DR14_L: 0x%08x  DR14_H: 0x%08x", esf->dr14_l, esf->dr14_h);
	EXCEPTION_DUMP(" DR15_L: 0x%08x  DR15_H: 0x%08x", esf->dr15_l, esf->dr15_h);
	EXCEPTION_DUMP(" DPSW:   0x%08x  DCMR:   0x%08x  DECNT:  0x%08x",
			esf->dpsw, esf->dcmr, esf->decnt);
#endif

	EXCEPTION_DUMP(" ACC0_L: 0x%08x  ACC0_H: 0x%08x  ACC0_G: 0x%08x",
			esf->acc0_l, esf->acc0_h, esf->acc0_g);
	EXCEPTION_DUMP(" ACC1_L: 0x%08x  ACC1_H: 0x%08x  ACC1_G: 0x%08x",
			esf->acc1_l, esf->acc1_h, esf->acc1_g);

#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
	EXCEPTION_DUMP(" FPSW:  0x%08x", esf->fpsw);
#endif

#endif

	EXCEPTION_DUMP(" r1:    0x%08x  r2:     0x%08x  r3:     0x%08x",
			esf->r1, esf->r2, esf->r3);
	EXCEPTION_DUMP(" r4:    0x%08x  r5:     0x%08x  r6:     0x%08x",
			esf->r4, esf->r5, esf->r6);
	EXCEPTION_DUMP(" r7:    0x%08x  r8:     0x%08x  r9:     0x%08x",
			esf->r7, esf->r8, esf->r9);
	EXCEPTION_DUMP(" r10:   0x%08x  r11:    0x%08x  r12:    0x%08x",
			esf->r10, esf->r11, esf->r12);
	EXCEPTION_DUMP(" r13:   0x%08x  r14:    0x%08x  r15:    0x%08x",
			esf->r13, esf->r14, esf->r15);
	EXCEPTION_DUMP(" PC:    0x%08x  PSW:    0x%08x", esf->entry_point, esf->psw);

}
#endif

void z_rx_fatal_error(unsigned int reason, const struct arch_esf *esf)
{
#ifdef CONFIG_EXCEPTION_DEBUG
	if (esf != NULL) {
		dump_rx_esf(esf);
	}
#endif /* CONFIG_EXCEPTION_DEBUG */

	z_fatal_error(reason, esf);
}
FUNC_NORETURN void arch_system_halt(unsigned int reason)
{
	ARG_UNUSED(reason);

	__asm__("brk");

	CODE_UNREACHABLE;
}
