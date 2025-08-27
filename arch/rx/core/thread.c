/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <kernel_internal.h>
#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(os, CONFIG_KERNEL_LOG_LEVEL);

/* variables to store the arguments of z_rx_context_switch_isr() (zephyr\arch\rx\core\switch.S)
 * when performing a cooperative thread switch. In that case, z_rx_context_switch_isr() triggerss
 * unmaskable interrupt 1 to actually perform the switch. The ISR to interrupt 1
 * (switch_isr_wrapper()) reads the arguments from these variables.
 */
void *coop_switch_to;
void **coop_switched_from;

void arch_new_thread(struct k_thread *thread, k_thread_stack_t *stack, char *stack_ptr,
			 k_thread_entry_t entry, void *arg1, void *arg2, void *arg3)
{
	struct arch_esf *iframe;

	iframe = Z_STACK_PTR_TO_FRAME(struct arch_esf, stack_ptr);

	/* initial value for the PSW (bits U and I are set) */
	iframe->psw = 0x30000;
	/* the initial entry point is the function z_thread_entry */
	iframe->entry_point = (uint32_t)z_thread_entry;
	/* arguments for the call of z_thread_entry (to be written to r1-r4) */
	iframe->r1 = (uint32_t)entry;
	iframe->r2 = (uint32_t)arg1;
	iframe->r3 = (uint32_t)arg2;
	iframe->r4 = (uint32_t)arg3;
	/* for debugging: */
	iframe->r5 = 5;
	iframe->r6 = 6;
	iframe->r7 = 7;
	iframe->r8 = 8;
	iframe->r9 = 9;
	iframe->r10 = 10;
	iframe->r11 = 11;
	iframe->r12 = 12;
	iframe->r13 = 13;
	iframe->r14 = 14;
	iframe->r15 = 15;

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
	iframe->fpsw = 16;
#endif
	iframe->acc0_l = 17;
	iframe->acc0_h = 18;
	iframe->acc0_g = 19;
	iframe->acc1_l = 20;
	iframe->acc1_h = 21;
	iframe->acc1_g = 22;
#endif

#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING) && defined(CONFIG_DFPU))
	iframe->dpsw = 23;
	iframe->dcmr = 24;
	iframe->decnt = 25;
	iframe->dr0_l = 26;
	iframe->dr0_h = 27;
	iframe->dr1_l = 28;
	iframe->dr1_h = 29;
	iframe->dr2_l = 30;
	iframe->dr2_h = 31;
	iframe->dr3_l = 32;
	iframe->dr3_h = 33;
	iframe->dr4_l = 34;
	iframe->dr4_h = 35;
	iframe->dr5_l = 36;
	iframe->dr5_h = 37;
	iframe->dr6_l = 38;
	iframe->dr6_h = 39;
	iframe->dr7_l = 40;
	iframe->dr7_h = 41;
	iframe->dr8_l = 42;
	iframe->dr8_h = 43;
	iframe->dr9_l = 44;
	iframe->dr9_h = 45;
	iframe->dr10_l = 46;
	iframe->dr10_h = 47;
	iframe->dr11_l = 48;
	iframe->dr11_h = 49;
	iframe->dr12_l = 50;
	iframe->dr12_h = 51;
	iframe->dr13_l = 52;
	iframe->dr13_h = 53;
	iframe->dr14_l = 54;
	iframe->dr14_h = 55;
	iframe->dr15_l = 56;
	iframe->dr15_h = 57;
#endif

#if defined(CONFIG_CPU_RXV1)
#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
	iframe->fpsw = 16;
#endif
	iframe->acc_l = 17;
	iframe->acc_h = 18;
#endif

	thread->switch_handle = (void *)iframe;
}

#if (defined(CONFIG_FPU) && defined(CONFIG_FPU_SHARING))
int arch_float_disable(struct k_thread *thread)	
{
	/* This is not supported. */
	return -ENOTSUP;
}

int arch_float_enable(struct k_thread *thread, unsigned int options)
{
	/* This is not supported. */
	return -ENOTSUP;
}
#endif /* CONFIG_FPU && CONFIG_FPU_SHARING */

int arch_coprocessors_disable(struct k_thread *thread)
{
	return -ENOTSUP;
}
