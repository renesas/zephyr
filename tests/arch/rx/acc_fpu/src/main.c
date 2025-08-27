/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "zephyr/kernel.h"
#include <stdint.h>
#include <zephyr/ztest.h>
#include <soc.h>
#include <zephyr/arch/exception.h>

#define STACK_SIZE (512)

ZTEST_SUITE(rx_acc_tests, NULL, NULL, NULL, NULL, NULL);

/*local variables*/
static K_THREAD_STACK_DEFINE(tstack_thread_1, STACK_SIZE);
static K_THREAD_STACK_DEFINE(tstack_thread_2, STACK_SIZE);
static struct k_thread thread_1;
static struct k_thread thread_2;
struct k_event my_event;
#if defined(CONFIG_CPU_RXV1)
static volatile int32_t thread_1_m, thread_1_h, thread_2_m, thread_2_h;
#elif (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
static volatile int32_t thread_1_acc0_l, thread_1_acc0_h, thread_1_acc0_g, thread_1_acc1_l,
	thread_1_acc1_h, thread_1_acc1_g, thread_1_fpsw;
static volatile int32_t thread_2_acc0_l, thread_2_acc0_h, thread_2_acc0_g, thread_2_acc1_l,
	thread_2_acc1_h, thread_2_acc1_g, thread_2_fpsw;
#endif
static struct arch_esf *thread_1_save;

static void thread_1_entry(void *p1, void *p2, void *p3)
{

	volatile int32_t thread_1_dpuf;

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
#if defined(CONFIG_DFPU)
	/* Set the initial accumulator registers */
	__asm volatile(
		"MOV #0x90ABCDEF,R15							\n" 
		/* Accumulator low 32 bits. */
		"MVTACLO	R15, A0						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator low 32 bits. */
		"MVTACLO	R15, A1						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A1						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A1						\n"
		"MOV #0x7FFFFFFF,R15								\n"
		"MVTC		R15, FPSW					\n"
		"DMOV.D #0xFFFFFFFF, DRH0\n"
		"DMOV.L #0xDDDDDDDD, DRL0\n"
		"DMOV.D #0xFFFFFFFF, DRH1\n"
		"DMOV.L #0xDDDDDDDD, DRL1\n"
		"DMOV.D #0xFFFFFFFF, DRH2\n"
		"DMOV.L #0xDDDDDDDD, DRL2\n"
		"DMOV.D #0xFFFFFFFF, DRH3\n"
		"DMOV.L #0xDDDDDDDD, DRL3\n"
		"DMOV.D #0xFFFFFFFF, DRH4\n"
		"DMOV.L #0xDDDDDDDD, DRL4\n"
		"DMOV.D #0xFFFFFFFF, DRH5\n"
		"DMOV.L #0xDDDDDDDD, DRL5\n"
		"DMOV.D #0xFFFFFFFF, DRH6\n"
		"DMOV.L #0xDDDDDDDD, DRL6\n"
		"DMOV.D #0xFFFFFFFF, DRH7\n"
		"DMOV.L #0xDDDDDDDD, DRL7\n"
		"DMOV.D #0xFFFFFFFF, DRH8\n"
		"DMOV.L #0xDDDDDDDD, DRL8\n"
		"DMOV.D #0xFFFFFFFF, DRH9\n"
		"DMOV.L #0xDDDDDDDD, DRL9\n"
		"DMOV.D #0xFFFFFFFF, DRH10\n"
		"DMOV.L #0xDDDDDDDD, DRL10\n"
		"DMOV.D #0xFFFFFFFF, DRH11\n"
		"DMOV.L #0xDDDDDDDD, DRL11\n"
		"DMOV.D #0xFFFFFFFF, DRH12\n"
		"DMOV.L #0xDDDDDDDD, DRL12\n"
		"DMOV.D #0xFFFFFFFF, DRH13\n"
		"DMOV.L #0xDDDDDDDD, DRL13\n"
		"DMOV.D #0xFFFFFFFF, DRH14\n"
		"DMOV.L #0xDDDDDDDD, DRL14\n"
		"DMOV.D #0xFFFFFFFF, DRH15\n"
		"DMOV.L #0xDDDDDDDD, DRL15\n"
		"MOV #0xFC007DFF,R15							\n"
		"MVTDC R15, DPSW\n"
		"MOV #0x00000001,R15							\n"
		"MVTDC R15, DCMR\n"
		"MOV #0x00010001,R15							\n"
		"MVTDC R15, DECNT\n");
#else
	/* Set the initial accumulator registers */
	__asm volatile(
		"MOV #0x90ABCDEF,R15							\n" 
		/* Accumulator low 32 bits. */
		"MVTACLO	R15, A0						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator low 32 bits. */
		"MVTACLO	R15, A1						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A1						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A1						\n"
		"MOV #0x7FFFFFFF,R15								\n"
		"MVTC		R15, FPSW					\n");
#endif
#elif defined(CONFIG_CPU_RXV1)
	/* Set the initial accumulator registers */
	__asm volatile("MOV #0x90ABCDEF,R15\n"
			"MVTACLO	R15\n"
			"MOV #0x12345678,R15\n"
			"MVTACHI	R15\n");
#endif

	thread_1_save = (struct arch_esf *)thread_1.switch_handle;

	/* yield the current thread. */
	k_yield();

	while (1) {
#if defined(CONFIG_CPU_RXV1)
		/* Get Accumulator registers */
		__asm volatile("MVFACMI	R15	\n"
			       "MOV.L R15, %0"
			       : "=r"(thread_1_m));

		__asm volatile("MVFACHI	R15	\n"
			       "MOV.L R15, %0"
			       : "=r"(thread_1_h));

		zassert_equal(thread_1_m, 0x567890AB, "Failed thread_1_m");
		zassert_equal(thread_1_h, 0x12345678, "Failed thread_1_h");
#endif

		__asm volatile(
			/* Save the FPSW and accumulator. */
			"MVFC		FPSW, R15					\n"
			"MOV.L R15, %0							\n"
			: "=r"(thread_1_fpsw));

		zassert_equal(thread_1_fpsw, 0xfc007d03, "Failed thread_1_fpsw");

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
		__asm volatile("MVFACGU	#0, A1, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_1_acc1_g));

		zassert_equal(thread_1_acc1_g & 0x000000FF, 0xEF, "Failed thread_1_acc1_g");

		__asm volatile("MVFACHI	#0, A1, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_1_acc1_h));

		zassert_equal(thread_1_acc1_h, 0x12345678, "Failed thread_1_acc1_h");

		__asm volatile(
			/* Low order word. */
			"MVFACLO	#0, A1, R15					\n"
			"MOV.L R15, %0							\n"
			: "=r"(thread_1_acc1_l));

		zassert_equal(thread_1_acc1_l, 0x90ABCDEF, "Failed thread_1_acc1_l");

		__asm volatile("MVFACGU	#0, A0, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_1_acc0_g));

		zassert_equal(thread_1_acc0_g & 0x000000FF, 0xEF, "Failed thread_1_acc0_g");

		__asm volatile("MVFACHI	#0, A0, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_1_acc0_h));

		zassert_equal(thread_1_acc0_h, 0x12345678, "Failed thread_1_acc0_h");

		__asm volatile(
			/* Low order word. */
			"MVFACLO	#0, A0, R15					\n"
			"MOV.L R15, %0							\n"
			: "=r"(thread_1_acc0_l));

		zassert_equal(thread_1_acc0_l, 0x90ABCDEF, "Failed thread_1_acc0_l");

		/* yield the current thread. */
		k_yield();

#if defined(CONFIG_DFPU)
		__asm volatile("DMOV.L DRH0, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH0");

		__asm volatile("DMOV.L DRL0, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL0");

		__asm volatile("DMOV.L DRH1, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH1");

		__asm volatile("DMOV.L DRL1, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL1");

		__asm volatile("DMOV.L DRH2, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH2");

		__asm volatile("DMOV.L DRL2, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL2");

		__asm volatile("DMOV.L DRH3, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH3");

		__asm volatile("DMOV.L DRL3, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL3");

		__asm volatile("DMOV.L DRH4, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH4");

		__asm volatile("DMOV.L DRL4, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL4");

		__asm volatile("DMOV.L DRH5, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH5");

		__asm volatile("DMOV.L DRL5, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL5");

		__asm volatile("DMOV.L DRH6, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH6");

		__asm volatile("DMOV.L DRL6, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL6");

		__asm volatile("DMOV.L DRH7, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH7");

		__asm volatile("DMOV.L DRL7, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL7");

		__asm volatile("DMOV.L DRH8, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH8");

		__asm volatile("DMOV.L DRL8, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL8");

		__asm volatile("DMOV.L DRH9, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH9");

		__asm volatile("DMOV.L DRL9, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL9");

		__asm volatile("DMOV.L DRH10, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH10");

		__asm volatile("DMOV.L DRL10, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL10");

		__asm volatile("DMOV.L DRH11, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH11");

		__asm volatile("DMOV.L DRL11, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL11");

		__asm volatile("DMOV.L DRH12, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH12");

		__asm volatile("DMOV.L DRL12, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL12");

		__asm volatile("DMOV.L DRH13, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH13");

		__asm volatile("DMOV.L DRL13, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL13");

		__asm volatile("DMOV.L DRH14, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH14");

		__asm volatile("DMOV.L DRL14, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL14");

		__asm volatile("DMOV.L DRH15, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xFFFFFFFF, "Failed thread_1_DRH15");

		__asm volatile("DMOV.L DRL15, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xDDDDDDDD, "Failed thread_1_DRL15");

		__asm volatile("MVFDC DPSW, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0xfc007d03, "Failed thread_1 DPSW");

		__asm volatile("MVFDC DCMR, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0x1, "Failed thread_1 DCMR");

		__asm volatile("MVFDC DECNT, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_1_dpuf));
		zassert_equal(thread_1_dpuf, 0x10001, "Failed thread_1 DECNT");
#endif
#endif

		k_sleep(K_SECONDS(10U));

		/* Kill this task. */
		k_thread_abort(k_current_get());
	}
}

static void thread_2_entry(void *p1, void *p2, void *p3)
{
	volatile int32_t thread_2_dpuf;

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
#if defined(CONFIG_DFPU)
	/* Set the initial accumulator registers */
	__asm volatile(
		"MOV #0x90ABCDEF,R15							\n" 
		/* Accumulator low 32 bits.     */
		"MVTACLO	R15, A0						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator low 32 bits. */
		"MVTACLO	R15, A1						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A1						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A1						\n"
		"MOV #0xFFFFFFFF,R15								\n"
		"MVTC		R15, FPSW					\n"
		"DMOV.D #0xEEEEEEEE, DRH0\n"
		"DMOV.L #0xCCCCCCCC, DRL0\n"
		"DMOV.D #0xEEEEEEEE, DRH1\n"
		"DMOV.L #0xCCCCCCCC, DRL1\n"
		"DMOV.D #0xEEEEEEEE, DRH2\n"
		"DMOV.L #0xCCCCCCCC, DRL2\n"
		"DMOV.D #0xEEEEEEEE, DRH3\n"
		"DMOV.L #0xCCCCCCCC, DRL3\n"
		"DMOV.D #0xEEEEEEEE, DRH4\n"
		"DMOV.L #0xCCCCCCCC, DRL4\n"
		"DMOV.D #0xEEEEEEEE, DRH5\n"
		"DMOV.L #0xCCCCCCCC, DRL5\n"
		"DMOV.D #0xEEEEEEEE, DRH6\n"
		"DMOV.L #0xCCCCCCCC, DRL6\n"
		"DMOV.D #0xEEEEEEEE, DRH7\n"
		"DMOV.L #0xCCCCCCCC, DRL7\n"
		"DMOV.D #0xEEEEEEEE, DRH8\n"
		"DMOV.L #0xCCCCCCCC, DRL8\n"
		"DMOV.D #0xEEEEEEEE, DRH9\n"
		"DMOV.L #0xCCCCCCCC, DRL9\n"
		"DMOV.D #0xEEEEEEEE, DRH10\n"
		"DMOV.L #0xCCCCCCCC, DRL10\n"
		"DMOV.D #0xEEEEEEEE, DRH11\n"
		"DMOV.L #0xCCCCCCCC, DRL11\n"
		"DMOV.D #0xEEEEEEEE, DRH12\n"
		"DMOV.L #0xCCCCCCCC, DRL12\n"
		"DMOV.D #0xEEEEEEEE, DRH13\n"
		"DMOV.L #0xCCCCCCCC, DRL13\n"
		"DMOV.D #0xEEEEEEEE, DRH14\n"
		"DMOV.L #0xCCCCCCCC, DRL14\n"
		"DMOV.D #0xEEEEEEEE, DRH15\n"
		"DMOV.L #0xCCCCCCCC, DRL15\n"
		"MOV #0xFC007DFF,R15							\n"
		"MVTDC R15, DPSW\n"
		"MOV #0x00000001,R15							\n"
		"MVTDC R15, DCMR\n"
		"MOV #0x00010001,R15							\n"
		"MVTDC R15, DECNT\n");
#else
	/* Set the initial accumulator registers */
	__asm volatile(
		"MOV #0x90ABCDEF,R15							\n" 
		/* Accumulator low 32 bits.     */
		"MVTACLO	R15, A0						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A0						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator low 32 bits. */
		"MVTACLO	R15, A1						\n"
		"MOV #0x12345678,R15							\n"

		/* Accumulator high 32 bits. */
		"MVTACHI	R15, A1						\n"
		"MOV #0x90ABCDEF,R15							\n"

		/* Accumulator guard. */
		"MVTACGU	R15, A1						\n"
		"MOV #0xFFFFFFFF,R15								\n"
		"MVTC		R15, FPSW					\n");
#endif
#elif defined(CONFIG_CPU_RXV1)
	/* Set the initial accumulator registers */
	__asm volatile("MOV #0x23456789,R15\n"
			"MVTACLO	R15\n"
			"MOV #0xABCDEF01,R15\n"
			"MVTACHI	R15\n");
#endif

	/* yield the current thread. */
	k_yield();

	while (1) {
#if defined(CONFIG_CPU_RXV1)
		/* Get Accumulator registers */
		__asm volatile("MVFACMI	R15	\n"
			       "MOV.L R15, %0"
			       : "=r"(thread_2_m));
		__asm volatile("MVFACHI	R15	\n"
			       "MOV.L R15, %0"
			       : "=r"(thread_2_h));

		zassert_equal(thread_2_m, 0xEF012345, "Failed thread_2_m");
		zassert_equal(thread_2_h, 0xABCDEF01, "Failed thread_2_h");
#endif

		__asm volatile(
			/* Save the FPSW and accumulator. */
			"MVFC		FPSW, R15					\n"
			"MOV.L R15, %0							\n"
			: "=r"(thread_2_fpsw));

		zassert_equal(thread_2_fpsw, 0xfc007d03, "Failed thread_2_fpsw");

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
		__asm volatile("MVFACGU	#0, A1, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_2_acc1_g));

		zassert_equal(thread_2_acc1_g & 0x000000FF, 0xEF, "Failed thread_2_acc1_g");

		__asm volatile("MVFACHI	#0, A1, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_2_acc1_h));

		zassert_equal(thread_2_acc1_h, 0x12345678, "Failed thread_2_acc1_h");

		__asm volatile(
			/* Low order word. */
			"MVFACLO	#0, A1, R15					\n"
			"MOV.L R15, %0							\n"
			: "=r"(thread_2_acc1_l));

		zassert_equal(thread_2_acc1_l, 0x90ABCDEF, "Failed thread_2_acc1_l");

		__asm volatile("MVFACGU	#0, A0, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_2_acc0_g));

		zassert_equal(thread_2_acc0_g & 0x000000FF, 0xEF, "Failed thread_2_acc0_g");

		__asm volatile("MVFACHI	#0, A0, R15					\n"
			       "MOV.L R15, %0							\n"
			       : "=r"(thread_2_acc0_h));

		zassert_equal(thread_2_acc0_h, 0x12345678, "Failed thread_2_acc0_h");

		__asm volatile(
			/* Low order word. */
			"MVFACLO	#0, A0, R15					\n"
			"MOV.L R15, %0							\n"
			: "=r"(thread_2_acc0_l));

		zassert_equal(thread_2_acc0_l, 0x90ABCDEF, "Failed thread_2_acc0_l");

		/* yield the current thread. */
		k_yield();

#if defined(CONFIG_DFPU)
		__asm volatile("DMOV.L DRH0, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH0");

		__asm volatile("DMOV.L DRL0, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL0");

		__asm volatile("DMOV.L DRH1, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH1");

		__asm volatile("DMOV.L DRL1, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL1");

		__asm volatile("DMOV.L DRH2, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH2");

		__asm volatile("DMOV.L DRL2, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL2");

		__asm volatile("DMOV.L DRH3, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH3");

		__asm volatile("DMOV.L DRL3, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL3");

		__asm volatile("DMOV.L DRH4, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH4");

		__asm volatile("DMOV.L DRL4, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL4");

		__asm volatile("DMOV.L DRH5, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH5");

		__asm volatile("DMOV.L DRL5, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL5");

		__asm volatile("DMOV.L DRH6, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH6");

		__asm volatile("DMOV.L DRL6, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL6");

		__asm volatile("DMOV.L DRH7, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH7");

		__asm volatile("DMOV.L DRL7, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL7");

		__asm volatile("DMOV.L DRH8, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH8");

		__asm volatile("DMOV.L DRL8, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL8");

		__asm volatile("DMOV.L DRH9, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH9");

		__asm volatile("DMOV.L DRL9, R15 \n"
			       "MOV.L R15, %0    \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL9");

		__asm volatile("DMOV.L DRH10, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH10");

		__asm volatile("DMOV.L DRL10, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL10");

		__asm volatile("DMOV.L DRH11, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH11");

		__asm volatile("DMOV.L DRL11, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL11");

		__asm volatile("DMOV.L DRH12, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH12");

		__asm volatile("DMOV.L DRL12, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL12");

		__asm volatile("DMOV.L DRH13, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH13");

		__asm volatile("DMOV.L DRL13, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL13");

		__asm volatile("DMOV.L DRH14, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH14");

		__asm volatile("DMOV.L DRL14, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL14");

		__asm volatile("DMOV.L DRH15, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xEEEEEEEE, "Failed thread_1_DRH15");

		__asm volatile("DMOV.L DRL15, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xCCCCCCCC, "Failed thread_1_DRL15");

		__asm volatile("MVFDC DPSW, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0xfc007d03, "Failed thread_2 DPSW");

		__asm volatile("MVFDC DCMR, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0x1, "Failed thread_2 DCMR");

		__asm volatile("MVFDC DECNT, R15 \n"
			       "MOV.L R15, %0     \n"
			       : "=r"(thread_2_dpuf));
		zassert_equal(thread_2_dpuf, 0x10001, "Failed thread_2 DECNT");
#endif
#endif

		k_sleep(K_SECONDS(10U));

		/* Kill this task. */
		k_thread_abort(k_current_get());
	}
}

/**
 * @brief Test switching accumulator
 *
 * This test verifies various assert macros provided by ztest.
 *
 */
ZTEST(rx_acc_tests, test_counting_value)
{

	k_tid_t tid_1 = k_thread_create(&thread_1, tstack_thread_1, STACK_SIZE, thread_1_entry,
					NULL, NULL, NULL, K_PRIO_COOP(1), 0, K_NO_WAIT);

	k_tid_t tid_2 = k_thread_create(&thread_2, tstack_thread_2, STACK_SIZE, thread_2_entry,
					NULL, NULL, NULL, K_PRIO_COOP(1), 0, K_NO_WAIT);
}
