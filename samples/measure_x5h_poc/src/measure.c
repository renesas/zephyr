/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <zephyr/kernel.h>

#define NUMBER_OF_CORES_IN_CLUSTER 2

#define IPL_OK                    (0x6A6A0000u)
#define IPL_ERROR_FAULT_INJECTION (0xD2D24747u)

#define TMU_CLOCK_FREQUENCY                 (33u)
#define COUNTER_TO_US_DIVISOR               (1000000u)
#define US_TO_MS_DIVISOR                    (1000u)
#define US_TO_MS_DECIMAL_DIVISOR            (100u)
#define TMU_COUNTER_SOC_RESET_INITIAL_VALUE (0xFFFFFFFFuL)

#define TMU_TMU1_BASE (0xC0680000uL)
#define TMU_TCNT10    (TMU_TMU1_BASE + (0x0000000CuL))

#define MFIS_TARGET_HSCIF (0x00000005u)
#define LOG_PREFIX        "Zephyr "
#define NULL_CHAR         '\0'

#define HSFSR_TEND_CHECK (0x0000u)
#define HSFSR_TEND_MASK  (0x0040u)
#define HSFSR_TDFE_CHECK (0x0000u)
#define HSFSR_TDFE_MASK  (0x0020u)

#define HSCIF_0_BASE  (0xC0710000uL)
#define HSCIF_HSFSR0  (HSCIF_0_BASE + (0x00000010uL))
#define HSCIF_HSFTDR0 (HSCIF_0_BASE + (0x0000000CuL))

#define MFISLCKR_LCK_NOT_ACQUIRED_CHECK (0x00000000uL)
#define MFISLCKR_LCK_MASK               (0x00000001uL)
#define MFISWACNTR_CODEVALUE_SET        (0xACCE0000uL)
#define MFISWACNTR_REGISTERADDRESS_MASK (0x0000FFFFuL)
#define MFISLCKR_LCK_RELEASE_SET        (0x00000000uL)

#define MFIS_COMMON_BASE (0x189E1000uL)
#define MFIS_MFISLCKR0   (MFIS_COMMON_BASE + (0x000000C0uL))
#define MFIS_MFISLCKR1   (MFIS_COMMON_BASE + (0x000000C4uL))
#define MFIS_MFISLCKR2   (MFIS_COMMON_BASE + (0x000000C8uL))
#define MFIS_MFISLCKR3   (MFIS_COMMON_BASE + (0x000000CCuL))
#define MFIS_MFISLCKR4   (MFIS_COMMON_BASE + (0x000000D0uL))
#define MFIS_MFISLCKR5   (MFIS_COMMON_BASE + (0x000000D4uL))
#define MFIS_MFISLCKR6   (MFIS_COMMON_BASE + (0x000000D8uL))
#define MFIS_MFISLCKR7   (MFIS_COMMON_BASE + (0x000000DCuL))
#define MFIS_MFISWACNTR  (MFIS_COMMON_BASE + (0x00000904uL))

#define MFIS_TARGET_HSCIF (0x00000005u)

#define BIT_MASK_31_24     (0xFF000000uL)
#define BIT_MASK_23_16     (0x00FF0000uL)
#define BIT_MASK_15_8      (0x0000FF00uL)
#define BIT_MASK_7_0       (0x000000FFuL)
#define BIT_MASK_15_0      (0x0000FFFFuL)
#define BIT_MASK_64BIT_7_0 (0x00000000000000FFuLL)

#define BIT_SHIFT_24 (0x00000018uL)
#define BIT_SHIFT_16 (0x00000010uL)
#define BIT_SHIFT_8  (0x00000008uL)
#define BIT_SHIFT_1  (0x00000001uL)

#define DECIMAL_BASE (10u)
#define CHAR_ZERO    ('0')

#define TIME(...)  Rl_PrintToSerialOutput("T:" __VA_ARGS__)
#define ERROR(...) Rl_PrintToSerialOutput("E:" __VA_ARGS__)

const uint32_t mfis_mfislckr_table[] = {
	MFIS_MFISLCKR0, /* MFIS_RPC           */
	MFIS_MFISLCKR1, /* Reserved           */
	MFIS_MFISLCKR2, /* MFIS_UFS           */
	MFIS_MFISLCKR3, /* MFIS_OVERLAP_CHECK */
	MFIS_MFISLCKR4, /* MFIS_MEASUREMENT   */
	MFIS_MFISLCKR5  /* MFIS_TARGET_HSCIF  */
};

static uint8_t cluster_id;
static uint8_t core_id;

uint32_t Rd_LockMfis(uint32_t target)
{
	uint32_t register_value;

	for (;;) {
		register_value = sys_read32(mfis_mfislckr_table[target]);
		if (MFISLCKR_LCK_NOT_ACQUIRED_CHECK == (register_value & (MFISLCKR_LCK_MASK))) {
			break;
		}
	}

	return 0;
}

uint32_t Rd_UnlockMfis(uint32_t target)
{
	uint32_t register_value_mfis_mfiswacntr;
	uint32_t register_value_mfis_mfislckr;

	register_value_mfis_mfiswacntr = MFISWACNTR_CODEVALUE_SET;
	register_value_mfis_mfiswacntr |=
		(mfis_mfislckr_table[target] & MFISWACNTR_REGISTERADDRESS_MASK);

	register_value_mfis_mfislckr = sys_read32(mfis_mfislckr_table[target]);
	register_value_mfis_mfislckr &= (~(MFISLCKR_LCK_MASK));
	register_value_mfis_mfislckr |= MFISLCKR_LCK_RELEASE_SET;

	/* Write Access Control Register */
	/* MFISLCKR[j] Register address setting */
	sys_write32(register_value_mfis_mfiswacntr, MFIS_MFISWACNTR);

	/* MFIS Lock Register [j] (MFISLCKR[j]) */
	sys_write32(register_value_mfis_mfislckr, mfis_mfislckr_table[target]);

	return 0;
}

uint32_t R_UTILS_GetCpuID(void)
{
	uint32_t cpuid = __get_MPIDR();

	cpuid = ((cpuid >> 8) & 0xff) * NUMBER_OF_CORES_IN_CLUSTER + (cpuid & 0xff);

	return cpuid;
}

static inline uint32_t Rl_BitShiftRight32(uint32_t value, uint32_t shift_number)
{
	uint32_t i;

	if ((0u != value) && (0u != shift_number)) {
		for (i = 0u; i < shift_number; i++) {
			value /= 2u;
		}
	} else {
		/* do nothing */
	}

	return value;
}

static inline uint64_t Rl_BitShiftRight64(uint64_t value, uint32_t shift_number)
{
	uint32_t i;

	if ((0u != value) && (0u != shift_number)) {
		for (i = 0u; i < shift_number; i++) {
			value /= 2u;
		}
	} else {
		/* do nothing */
	}

	return value;
}

static inline void Rd_MmioWrite32(uintptr_t address, uint32_t data)
{
	*(volatile uint32_t *)address = data;
}

static inline void Rd_MmioWrite16(uintptr_t address, uint16_t data)
{
	*(volatile uint16_t *)address = data;
}

static inline void Rd_MmioWrite8(uintptr_t address, uint8_t data)
{
	*(volatile uint8_t *)address = data;
}

static inline uint32_t Rd_MmioRead32(uintptr_t address)
{
	return (*(volatile uint32_t *)address);
}

static inline uint16_t Rd_MmioRead16(uintptr_t address)
{
	return (*(volatile uint16_t *)address);
}

static inline uint8_t Rd_MmioRead8(uintptr_t address)
{
	return (*(volatile uint8_t *)address);
}

void Rd_PutConsole(uint8_t output_character)
{
	uint32_t register_value = 0uL;

	/* TDFE check */
	do {
		register_value = Rd_MmioRead16(HSCIF_HSFSR0);
	} while (HSFSR_TDFE_CHECK == (HSFSR_TDFE_MASK & register_value));

	Rd_MmioWrite8(HSCIF_HSFTDR0, output_character); /* Transfer one character */

	/* TDFE clear */
	register_value = Rd_MmioRead16(HSCIF_HSFSR0);
	register_value &= ~HSFSR_TDFE_MASK;
	Rd_MmioWrite16(HSCIF_HSFSR0, register_value);
}

static uint32_t rl_str_print(const char *str)
{
	uint32_t count = 0;

	while (*str != NULL_CHAR) {
		Rd_PutConsole((uint8_t)*str);
		str++;
		count++;
	}

	return count;
}

static void rl_output_cluster_core_id(void)
{
	uint32_t mpidr = 0u;

	static uint32_t count;

	if (count == 0u) {
		__asm__ volatile("mrc p15, 0, %0, c0, c0, 5" : "=r"(mpidr));
		/* MPIDR[15:8] = Cluster ID */
		cluster_id = (uint8_t)(Rl_BitShiftRight32(mpidr, BIT_SHIFT_8) & BIT_MASK_7_0);
		/* Change from integer to character */
		cluster_id = (cluster_id % DECIMAL_BASE) + CHAR_ZERO;

		/* MPIDR[7:0]  = Core ID */
		core_id = (uint8_t)(mpidr & BIT_MASK_7_0);
		/* Change from integer to character */
		core_id = (core_id % DECIMAL_BASE) + CHAR_ZERO;

		count = 1u;
	}
	Rd_PutConsole(cluster_id);
	rl_str_print("_");
	Rd_PutConsole(core_id);
	rl_str_print(" ");
}

static uint32_t rl_uint32_hex_print(uint32_t number)
{
	uint32_t i;
	uint32_t count = 0u;
	uint8_t character;

	for (i = 0u; i < 8u; i++) {
		character = (uint8_t)(Rl_BitShiftRight32(number, ((7u - i) * 4u)) & 0x0Fu);

		/* 0-F */
		if (character >= 0x0Au) {
			/* A-F */
			character += (uint8_t)('a' - 0x0Au);
		} else {
			/* 0-9 */
			character += (uint8_t)'0';
		}
		Rd_PutConsole(character);
		count++;
	}

	return count;
}

static uint32_t rl_uint64_hex_print(uint64_t number)
{
	uint32_t i;
	uint32_t count = 0u;
	uint8_t character;

	for (i = 0u; i < 16u; i++) {
		character = (uint8_t)(Rl_BitShiftRight64(number, ((15u - i) * 4u)) & 0x0Fu);

		/* 0-f */
		if (character >= 0x0Au) {
			/* a-f */
			character += (uint8_t)('a' - 0x0Au);
		} else {
			/* 0-9 */
			character += (uint8_t)'0';
		}
		Rd_PutConsole(character);
		count++;
	}

	return count;
}

static uint32_t rl_uint32_print(uint32_t number)
{
	uint8_t number_buf[10];
	uint32_t count = 0U;
	uint32_t i = 0U;
	uint32_t rem;
	uint32_t number_u32 = number;

	while (1) {
		rem = number_u32 % 10u;
		number_buf[i] = (uint8_t)('0' + rem);

		i++;
		number_u32 = number_u32 / 10u;
		if (1u > number_u32) {
			break;
		}
	}

	while (i != 0U) {
		i--;
		Rd_PutConsole(number_buf[i]);
		count++;
	}

	return count;
}

void Rl_CheckTendFlag(void)
{
	uint32_t register_value = 0uL;

	/* TEND check */
	do {
		register_value = Rd_MmioRead16(HSCIF_HSFSR0);
	} while (HSFSR_TEND_CHECK == (HSFSR_TEND_MASK & register_value));

	/* TEND clear */
	register_value = Rd_MmioRead16(HSCIF_HSFSR0);
	register_value &= ~HSFSR_TEND_MASK;
	Rd_MmioWrite16(HSCIF_HSFSR0, register_value);
}

void Rl_PrintToSerialOutput(const char *p_format, ...)
{
	va_list arguments;
	uint32_t number_u32;
	uint64_t number_u64;
	char *p_str;
	uint32_t count = 0u;

	/* Lock to avoid conflict for HSCIF */
	Rd_LockMfis(MFIS_TARGET_HSCIF);

	rl_str_print(LOG_PREFIX);

	rl_output_cluster_core_id();

	va_start(arguments, p_format);
	while (NULL_CHAR != *p_format) {
		if ('%' == *p_format) {
			p_format++;
			switch (*p_format) {
			case 's':
				p_str = va_arg(arguments, char *);
				count += rl_str_print(p_str);
				break;
			case 'x':
				number_u32 = va_arg(arguments, uint32_t);
				count += rl_uint32_hex_print(number_u32);
				break;
			case 'a':
				number_u64 = va_arg(arguments, uint64_t);
				count += rl_uint64_hex_print(number_u64);
				break;
			case 'd':
				number_u32 = va_arg(arguments, uint32_t);
				count += rl_uint32_print(number_u32);
				break;
			default:
				break;
			}
		} else {
			Rd_PutConsole((uint8_t)*p_format);
			count++;
		}
		p_format++;
	}
	va_end(arguments);

	Rl_CheckTendFlag();

	/* Unlock to avoid conflict for HSCIF */
	Rd_UnlockMfis(MFIS_TARGET_HSCIF);
}

uint32_t rl_ConvertTmuCounterToMs(uint32_t *tmu_counter_value, uint32_t *time_ms_integer,
				  uint32_t *time_ms_decimal)
{
	uint32_t return_value = IPL_ERROR_FAULT_INJECTION;
	uint32_t time_us;

	time_us = *tmu_counter_value / TMU_CLOCK_FREQUENCY;
	*time_ms_integer = time_us / US_TO_MS_DIVISOR;
	*time_ms_decimal = (time_us % US_TO_MS_DIVISOR) / US_TO_MS_DECIMAL_DIVISOR;
	return_value = IPL_OK;

	return return_value;
}

uint32_t Rd_GetTmuCounter(uint32_t *timer_count)
{
	uint32_t return_value = IPL_ERROR_FAULT_INJECTION;

	*timer_count = Rd_MmioRead32(TMU_TCNT10);
	return_value = IPL_OK;

	return return_value;
}

uint32_t Rl_SaveLapTimeFromSocReset(char *start_string_input)
{
	uint32_t return_value = IPL_ERROR_FAULT_INJECTION;
	uint32_t function_result = IPL_ERROR_FAULT_INJECTION;
	uint32_t time_count_current = 0u;
	uint32_t time_count_difference = 0u;
	uint32_t time_integer_current = 0u;
	uint32_t time_decimal_current = 0u;

	function_result = Rd_GetTmuCounter(&time_count_current);
	time_count_difference = TMU_COUNTER_SOC_RESET_INITIAL_VALUE - time_count_current;
	rl_ConvertTmuCounterToMs(&time_count_difference, &time_integer_current,
				 &time_decimal_current);
	if (IPL_OK == function_result) {
		TIME("%s: %d.%d ms\r\n", start_string_input, time_integer_current,
		     time_decimal_current);
		return_value = IPL_OK;
	} else {
		ERROR("Rd_GetTmuCounter() return 0x%x\r\n", function_result);
		return_value = function_result;
	}

	return return_value;
}
