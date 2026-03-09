/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>


#define __ext_ramfunc __attribute__((noinline)) __attribute__((section(".ext_ramfunc")))

extern uint8_t __ext_ram_memory_start[];
extern uint8_t __ext_ram_memory_end[];
extern uint8_t __ext_ram_memory_load_start[];

static int ram_data_preparation(void)
{
    size_t size = __ext_ram_memory_end - __ext_ram_memory_start;

    printk("Start preparing data in external RAM\n");

    memcpy(__ext_ram_memory_start,
           __ext_ram_memory_load_start,
           size);

    return 0;
}

SYS_INIT(ram_data_preparation, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

__ext_ramfunc void ext_ram_function(void)
{
    volatile uint32_t sum = 0;

    for (int i = 0; i < 10000; i++) {
        sum += i;
    }
}

__ramfunc void ram_function(void)
{
    volatile uint32_t sum = 0;

    for (int i = 0; i < 10000; i++) {
        sum += i;
    }
}

__attribute__((noinline)) void flash_function(void)
{
    volatile uint32_t sum = 0;

    for (int i = 0; i < 10000; i++) {
        sum += i;
    }
}

uint32_t measure_cycles(void (*func)(void))
{
	uint32_t start;
	uint32_t end;

	start = k_cycle_get_32();
	func();
	end = k_cycle_get_32();

	return end - start;
}

int main(void)
{
	uint32_t flash_cycles;
	uint32_t ram_cycles;
	uint32_t extram_cycles;

	printk("Function addresses:\n");
	printk("Flash function   : %p\n", flash_function);
	printk("SRAM function     : %p\n", ram_function);
	printk("OSPI External RAM func: %p\n", ext_ram_function);

	flash_cycles = measure_cycles(flash_function);
	ram_cycles = measure_cycles(ram_function);
	extram_cycles = measure_cycles(ext_ram_function);

	printk("\nExecution cycles:\n");
	printk("Flash       : %u cycles\n", flash_cycles);
	printk("Internal RAM: %u cycles\n", ram_cycles);
	printk("External RAM: %u cycles\n", extram_cycles);

	return 0;
}
