/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/coredump.h>
#include <zephyr/arch/rx/exception.h>

/* FPSW and the DPFPU registers are not captured yet, as struct arch_esf only saves them when
 * CONFIG_FPU_SHARING is enabled.
 */
#if defined(CONFIG_CPU_RXV1)
#define RX_ARCH_HDR_VER 1
#elif (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
#define RX_ARCH_HDR_VER 2
#else
#error "Unsupported RX CPU variant"
#endif

/* PSW.U: stack pointer select bit, set when the context ran on the USP */
#define RX_PSW_U BIT(17)

struct rx_arch_block {
	uint32_t sp;

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
	uint32_t acc0_l;
	uint32_t acc0_h;
	uint32_t acc0_g;
	uint32_t acc1_l;
	uint32_t acc1_h;
	uint32_t acc1_g;
#elif defined(CONFIG_CPU_RXV1)
	uint32_t acc_l;
	uint32_t acc_h;
#endif

	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r4;
	uint32_t r5;
	uint32_t r6;
	uint32_t r7;
	uint32_t r8;
	uint32_t r9;
	uint32_t r10;
	uint32_t r11;
	uint32_t r12;
	uint32_t r13;
	uint32_t r14;
	uint32_t r15;

	uint32_t pc;
	uint32_t psw;
} __packed;

static struct rx_arch_block arch_blk;

uint16_t arch_coredump_tgt_code_get(void)
{
	return COREDUMP_TGT_RX;
}

static uintptr_t rx_fault_sp_get(const struct arch_esf *esf)
{
	uint32_t usp;

	/* If the faulting context ran on the USP (PSW.U set), read it directly.
	 * Otherwise the context ran on the ISP and its SP is the address right above the pushed
	 * frame.
	 */
	if ((esf->psw & RX_PSW_U) != 0U) {
		__asm__ volatile("mvfc usp, %0" : "=r"(usp));
		return usp;
	}

	return (uintptr_t)esf + sizeof(struct arch_esf);
}

void arch_coredump_info_dump(const struct arch_esf *esf)
{
	struct coredump_arch_hdr_t hdr = {
		.id = COREDUMP_ARCH_HDR_ID,
		.hdr_version = RX_ARCH_HDR_VER,
		.num_bytes = sizeof(arch_blk),
	};

	if (esf == NULL) {
		return;
	}

	arch_blk.sp = (uint32_t)rx_fault_sp_get(esf);

#if (defined(CONFIG_CPU_RXV2) || defined(CONFIG_CPU_RXV3) || defined(CONFIG_CPU_RXV3E))
	arch_blk.acc0_l = esf->acc0_l;
	arch_blk.acc0_h = esf->acc0_h;
	arch_blk.acc0_g = esf->acc0_g;
	arch_blk.acc1_l = esf->acc1_l;
	arch_blk.acc1_h = esf->acc1_h;
	arch_blk.acc1_g = esf->acc1_g;
#elif defined(CONFIG_CPU_RXV1)
	arch_blk.acc_l = esf->acc_l;
	arch_blk.acc_h = esf->acc_h;
#endif

	arch_blk.r1 = esf->r1;
	arch_blk.r2 = esf->r2;
	arch_blk.r3 = esf->r3;
	arch_blk.r4 = esf->r4;
	arch_blk.r5 = esf->r5;
	arch_blk.r6 = esf->r6;
	arch_blk.r7 = esf->r7;
	arch_blk.r8 = esf->r8;
	arch_blk.r9 = esf->r9;
	arch_blk.r10 = esf->r10;
	arch_blk.r11 = esf->r11;
	arch_blk.r12 = esf->r12;
	arch_blk.r13 = esf->r13;
	arch_blk.r14 = esf->r14;
	arch_blk.r15 = esf->r15;

	arch_blk.pc = esf->entry_point;
	arch_blk.psw = esf->psw;

	coredump_buffer_output((uint8_t *)&hdr, sizeof(hdr));

	coredump_buffer_output((uint8_t *)&arch_blk, sizeof(arch_blk));
}
