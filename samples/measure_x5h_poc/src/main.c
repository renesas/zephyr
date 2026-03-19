/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>

#define STACKSIZE 2048
#define PRIORITY  0

uint32_t Rl_SaveLapTimeFromSocReset(char *start_string_input);
uint32_t R_UTILS_GetCpuID(void);

/* Static thread entry point */
void static_thread_func(void)
{
	Rl_SaveLapTimeFromSocReset("Zephyr static thread entry");

	/* Some work is being done */
	for (int i = 0; i <= 1000; i++) {
	};
}

/* Create static thread */
K_THREAD_DEFINE(static_thread, STACKSIZE, static_thread_func, NULL, NULL, NULL, PRIORITY, 0, 0);

/* Dynamic thread entry point */
void dynamic_thread_func(void)
{
	Rl_SaveLapTimeFromSocReset("Zephyr dynamic thread entry");

	/* Some work is being done */
	for (int i = 0; i <= 1000; i++) {
	};
}

K_THREAD_STACK_DEFINE(dynamic_thread_stack_area, STACKSIZE);
struct k_thread dynamic_thread_data;

int main(void)
{
	Rl_SaveLapTimeFromSocReset("Zephyr Main thread entry");
	printf("Zephyr Core %d: BSP R-Car X5H Zephyr\n", R_UTILS_GetCpuID());

	/* Create dynamic thread */
	k_thread_create(&dynamic_thread_data, dynamic_thread_stack_area,
			K_THREAD_STACK_SIZEOF(dynamic_thread_stack_area),
			(k_thread_entry_t)dynamic_thread_func, NULL, NULL, NULL, PRIORITY, 0,
			K_NO_WAIT);

	while (1) {
		printf("Zephyr Core %d: Main task running\n", R_UTILS_GetCpuID());

		/* Sleep for 2s */
		k_msleep(2000);
	}

	return 0;
}
