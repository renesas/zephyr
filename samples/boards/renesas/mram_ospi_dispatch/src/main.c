/*
 * Copyright (c) 2018 Intel Corporation.
 * Copyright (c) 2022 Carlo Caione <ccaione@baylibre.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

/*
 * This function will allow execute from sram region.  This is needed only for
 * this sample because by default all soc will disable the execute from SRAM.
 * An application that requires that the code be executed from SRAM will have
 * to configure the region appropriately in arm_mpu_regions.c.
 */
#ifdef CONFIG_ARM_MPU
#include <cmsis_core.h>
void disable_mpu_rasr_xn(void)
{
	uint32_t index;

	/*
	 * Kept the max index as 8(irrespective of soc) because the sram would
	 * most likely be set at index 2.
	 */
	for (index = 0U; index < 8; index++) {
		MPU->RNR = index;
#if defined(CONFIG_ARMV8_M_BASELINE) || defined(CONFIG_ARMV8_M_MAINLINE)
		if (MPU->RBAR & MPU_RBAR_XN_Msk) {
			MPU->RBAR ^= MPU_RBAR_XN_Msk;
		}
#else
		if (MPU->RASR & MPU_RASR_XN_Msk) {
			MPU->RASR ^= MPU_RASR_XN_Msk;
		}
#endif /* CONFIG_ARMV8_M_BASELINE || CONFIG_ARMV8_M_MAINLINE */
	}
}
#endif /* CONFIG_ARM_MPU */

extern void function_in_ext_flash(void);
extern void function_in_sram(void);
extern int ospi_image_verify(void);

int main(void)
{
#ifdef CONFIG_ARM_MPU
	disable_mpu_rasr_xn();
#endif	/* CONFIG_ARM_MPU */

	printk("Address of %s function %p\n", __func__, &main);

	/* Never execute code from the external flash unless its signature is valid */
	if (ospi_image_verify() != 0) {
		printk("External flash image verification FAILED, not executing it\n");
		while (1) {
			k_msleep(1000);
			printk("External flash image verification FAILED, not executing it\n");
		}
	}
	printk("External flash image verified\n");

	function_in_ext_flash();

	return 0;
}
