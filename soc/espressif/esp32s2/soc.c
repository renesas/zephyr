/*
 * Copyright (c) 2021 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include "soc.h"
#include <soc/rtc_cntl_reg.h>
#include <soc/timer_group_reg.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#include <xtensa/config/core-isa.h>
#include <xtensa/corebits.h>
#include <esp_private/spi_flash_os.h>
#include <esp_private/esp_mmu_map_private.h>
#include <esp_flash_internal.h>
#if CONFIG_ESP_SPIRAM
#include "psram.h"
#endif

#include <zephyr/kernel_structs.h>
#include <kernel_internal.h>
#include <string.h>
#include <zephyr/toolchain.h>
#include <zephyr/types.h>

#include <esp_private/system_internal.h>
#include <esp32s2/rom/cache.h>
#include <soc/gpio_periph.h>
#include <esp_cpu.h>
#include <hal/cpu_hal.h>
#include <hal/soc_hal.h>
#include <hal/wdt_hal.h>
#include <esp_timer.h>
#include <esp_err.h>
#include <esp_clk_internal.h>
#include <zephyr/sys/printk.h>
#include "esp_log.h"

#define TAG "boot.esp32s2"

extern void rtc_clk_cpu_freq_set_xtal(void);
extern void esp_reset_reason_init(void);
extern void z_prep_c(void);

/*
 * This is written in C rather than assembly since, during the port bring up,
 * Zephyr is being booted by the Espressif bootloader.  With it, the C stack
 * is already set up.
 */
void __attribute__((section(".iram1"))) __esp_platform_start(void)
{
	extern uint32_t _init_start;

	/* Move the exception vector table to IRAM. */
	__asm__ __volatile__("wsr %0, vecbase" : : "r"(&_init_start));

	/* Zero out BSS */
	z_bss_zero();

	/* Disable normal interrupts. */
	__asm__ __volatile__("wsr %0, PS" : : "r"(PS_INTLEVEL(XCHAL_EXCM_LEVEL) | PS_UM | PS_WOE));

	/* Initialize the architecture CPU pointer.  Some of the
	 * initialization code wants a valid arch_current_thread() before
	 * arch_kernel_init() is invoked.
	 */
	__asm__ __volatile__("wsr.MISC0 %0; rsync" : : "r"(&_kernel.cpus[0]));

	esp_reset_reason_init();

#ifndef CONFIG_MCUBOOT
	/* ESP-IDF 2nd stage bootloader enables RTC WDT to check on startup sequence
	 * related issues in application. Hence disable that as we are about to start
	 * Zephyr environment.
	 */
	wdt_hal_context_t rtc_wdt_ctx = {.inst = WDT_RWDT, .rwdt_dev = &RTCCNTL};

	wdt_hal_write_protect_disable(&rtc_wdt_ctx);
	wdt_hal_disable(&rtc_wdt_ctx);
	wdt_hal_write_protect_enable(&rtc_wdt_ctx);

	/*
	 * Configure the mode of instruction cache :
	 * cache size, cache associated ways, cache line size.
	 */
	esp_config_instruction_cache_mode();

	/*
	 * If we need use SPIRAM, we should use data cache, or if we want to
	 * access rodata, we also should use data cache.
	 * Configure the mode of data : cache size, cache associated ways, cache
	 * line size.
	 * Enable data cache, so if we don't use SPIRAM, it just works.
	 */
	esp_config_data_cache_mode();
	esp_rom_Cache_Enable_DCache(0);

	esp_timer_early_init();

	esp_mspi_pin_init();

	esp_flash_app_init();

	esp_mmu_map_init();

#if CONFIG_ESP_SPIRAM
	esp_init_psram();
#endif /* CONFIG_ESP_SPIRAM */

#endif /* !CONFIG_MCUBOOT */

	esp_intr_initialize();

#if CONFIG_ESP_SPIRAM
	/* Init Shared Multi Heap for PSRAM */
	int err = esp_psram_smh_init();

	if (err) {
		printk("Failed to initialize PSRAM shared multi heap (%d)\n", err);
	}
#endif

	/* Start Zephyr */
	z_prep_c();

	CODE_UNREACHABLE;
}

/* Boot-time static default printk handler, possibly to be overridden later. */
int IRAM_ATTR arch_printk_char_out(int c)
{
	if (c == '\n') {
		esp_rom_uart_tx_one_char('\r');
	}
	esp_rom_uart_tx_one_char(c);
	return 0;
}

void sys_arch_reboot(int type)
{
	esp_restart_noos();
}
