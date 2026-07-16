/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/linker/linker-defs.h>

/* Read-only lookup table placed in LRAM0 NS alias (VMA = 0x00000000+offset).
 * Simulates the constant data segment of a library linked at 0x00000000.
 * Uses ".itcm.rodata" — the linker script collects ".itcm.*" alongside ".itcm",
 * so both end up in the ITCM region. Code and rodata must use separate subsection
 * names because GCC assigns different ELF section flags to each (SHF_EXECINSTR
 * for code, none for rodata), and GCC rejects mixed flags in one section name.
 */
static const uint32_t lram_ns_table[] __attribute__((section(".itcm.rodata"))) = {
	0xDEADBEEF, 0xCAFEBABE, 0x12345678, 0xA5A5A5A5,
};

/* Function placed in LRAM0 NS alias — simulates a library routine that must
 * execute from address 0x00000000.
 */
static uint32_t __attribute__((section(".itcm.text"))) lram_ns_checksum(void)
{
	uint32_t acc = 0;

	for (size_t i = 0; i < ARRAY_SIZE(lram_ns_table); i++) {
		acc ^= lram_ns_table[i];
	}
	return acc;
}

int main(void)
{
	printk("LRAM0 Non-Secure Alias Demo\n");
	printk("===========================\n");

	/* __itcm_start must be 0x00000000 when itcm_ns is active */
	printk("__itcm_start      = %p\n", (void *)__itcm_start);
	printk("__itcm_end        = %p\n", (void *)__itcm_end);

	printk("\nlram_ns_table address  = %p\n", (void *)lram_ns_table);
	printk("lram_ns_checksum function address = %p\n", (void *)lram_ns_checksum);

	/* Both addresses must fall within [0x00000000, 0x00020000) */
	if ((uintptr_t)lram_ns_table >= 0x00020000U ||
	    (uintptr_t)lram_ns_checksum >= 0x00020000U) {
		printk("ERROR: objects not placed in NS LRAM region!\n");
		return -1;
	}

	/* Verify that the table was copied correctly from MRAM */
	if (lram_ns_table[0] != 0xDEADBEEFU ||
	    lram_ns_table[1] != 0xCAFEBABEU ||
	    lram_ns_table[2] != 0x12345678U ||
	    lram_ns_table[3] != 0xA5A5A5A5U) {
		printk("ERROR: table data mismatch — LRAM0 NS copy failed!\n");
		return -1;
	}

	printk("\nlram_ns_table contents:\n");
	for (size_t i = 0; i < ARRAY_SIZE(lram_ns_table); i++) {
		printk("  [%zu] = 0x%08X\n", i, lram_ns_table[i]);
	}

	/* Call the function executing from NS LRAM */
	uint32_t csum = lram_ns_checksum();

	printk("\nlram_ns_checksum() = 0x%08X\n", csum);

	/* Expected: 0xDEADBEEF ^ 0xCAFEBABE ^ 0x12345678 ^ 0xA5A5A5A5 */
	uint32_t expected = 0xDEADBEEFU ^ 0xCAFEBABEU ^ 0x12345678U ^ 0xA5A5A5A5U;

	if (csum != expected) {
		printk("ERROR: checksum mismatch (expected 0x%08X)!\n", expected);
		return -1;
	}

	printk("\nAll checks passed — LRAM0 non-secure alias is working.\n");
	return 0;
}
