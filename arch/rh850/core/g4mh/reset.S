/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/arch/rh850/g4mh/g4mh.h>

/* Definition of PE number */
#define PE0       0

  extern ___kernel_ram_end
  extern __vector_start
  extern __irq_vector_table
  extern _z_interrupt_stacks
  extern Region$$Table$$Base
  extern Region$$Table$$Limit
  extern __iar_static_base$$TP

  extern _z_prep_c
  extern _exit
  extern ___iar_data_init2
  section .text:CODE(2)

/* Common Startup Routine */
  public _z_rh850_reset
  public __start
__start:
_z_rh850_reset:
/* PE0 set Boot Control Register for all PE */
  stsr CPU_REG_PEID, r1, 2
  andi 0x7, r1, r1
  cmp PE0, r1
  bne ?_z_rh850_reset
  synci
  jr _BOOTCTRL_SETTING
?_z_rh850_reset:
  jr _PE_RESET

_PE_RESET:
/* CPU function group program registers initialization */
  mov r0, r2
  mov r0, r3
  mov r0, r4
  mov r0, r5
  mov r0, r6
  mov r0, r7
  mov r0, r8
  mov r0, r9
  mov r0, r10
  mov r0, r11
  mov r0, r12
  mov r0, r13
  mov r0, r14
  mov r0, r15
  mov r0, r16
  mov r0, r17
  mov r0, r18
  mov r0, r19
  mov r0, r20
  mov r0, r21
  mov r0, r22
  mov r0, r23
  mov r0, r24
  mov r0, r25
  mov r0, r26
  mov r0, r27
  mov r0, r28
  mov r0, r29
  mov r0, r30
  mov r0, r31

/* CPU function group system registers initialization */
  ldsr r0, EIPC
  ldsr r0, EIPSW
  ldsr r0, FEPC
  ldsr r0, FEPSW
  ldsr r0, EIIC
  ldsr r0, FEIC
  ldsr r0, CTPC
  ldsr r0, CTPSW
  ldsr r0, CTBP
  ldsr r0, EIWR
  ldsr r0, FEWR
  ldsr r0, RBASE
  ldsr r0, EBASE
  ldsr r0, INTBP
  ldsr r0, SCCFG
  ldsr r0, SCBP
  ldsr r0, MEA
  ldsr r0, MEI

/* Disable EI-level interrupt */
  mov 0x00000020, r6
  ldsr r6, PSW

/*
 *  Don't Initialize registers list
 *  Following registers is't set by this startup routine
 *  SNZCFG, SPID, SPIDLIST, MCTL, PID, SVLOCK,
 *  PEID, BMID, RBCR0, RBCR1, RBNR, RBIP
 */

/* Jump to PEn Startup Routine, currently only single-core is supported
 * so cores others than PE0 will jump to __wait_forever
 */
  stsr CPU_REG_PEID, r10, 2
  andi 0x7, r10, r10

	cmp	PE0, r10
	be __start_pe0

  public __wait_forever
__wait_forever:
  br __wait_forever

__start_pe0:
  jr _RESET_PE0

/* PEn activation */
_BOOTCTRL_SETTING:
  mov 1, r3
  shl CONFIG_MP_MAX_NUM_CPUS, r3
  add -1, r3
  mov  0xFFFB2000, r2
  st.w r3, 0[r2]
  jr   _PE_RESET

/* Initialize SRAM areas with a known value (0xaa) */
_Init_SRAM:
  mov 0xAAAAAAAA, r6
  mov CONFIG_SRAM_BASE_ADDRESS, r7
  mov ___kernel_ram_end, r8

_cram_loop:
  cmp r7, r8
  bne ?_cram_loop
  jmp [lp]
?_cram_loop:
  st.w r6, 0[r7]
  add 4, r7
  br _cram_loop

/* PE0 Specific Startup Routine */
  public _RESET_PE0
_RESET_PE0:
/* Memory Initialization */
/* Initialize SRAM */
  jarl _Init_SRAM, lp

/* PE0 Initialization */
/* Initialization of the global pointer */
  mov 0x00, gp

/* Initialization of the text pointer */
  mov __iar_static_base$$TP, tp

/* Initialization of the element pointer */
  mov 0x00, ep

/* Initialization of the stack pointer */
  mov _z_interrupt_stacks, r1
  mov CONFIG_ISR_STACK_SIZE, r6
  add r6, r1
  mov r1, sp

/* Initialization of the interrupt base pointer */
  mov __irq_vector_table , r1
  ldsr r1, INTBP

/* First set EBASE register address */
  mov __vector_start, r10
  ldsr r10, EBASE

/* Set PSW.EBV bit to 1 -> RBASE != EBASE */
  stsr CPU_REG_PSW, r10, 0            /* Read PSW register */
  mov PSW_EBV, r11                    /* PSW.EBV bit mask */
  or r11, r10                         /* Set EBV bit */
  ldsr r10, PSW                       /* Write back to PSW */

/* Use EIWR to save the previous EIIC value so it can be restored later; initialize EIWR to 0. */
  ldsr r0, EIWR

/* Initialization of the FPU */
#if defined(CONFIG_CPU_HAS_FPU) && defined(CONFIG_FPU)
	stsr CPU_REG_PSW, r10, 0		        /* Read current PSW */
	movhi	0x1, r0, r11                  /* PSW.CU0 bit (FPU enable) */
	or r11, r10                         /* Enable FPU in PSW */
	ldsr r10, PSW		                    /* Write back to PSW */

	movhi	0x2, r0, r11
	ldsr r11, CPU_REG_FPSR, 0		        /* initialize FPSR */
	ldsr r0, CPU_REG_FPEPC, 0		        /* initialize FPEPC */
#endif

/* Initialization of the FXU */
#if defined(CONFIG_CPU_HAS_FXU) && defined(CONFIG_FXU)
	stsr CPU_REG_PSW, r10, 0		        /* Read current PSW */
	movhi	0x2, r0, r11                  /* PSW.CU1 bit (FXU enable) */
	or r11, r10                         /* Enable FXU in PSW */
	ldsr r10, PSW		                    /* Write back to PSW */

	movhi	0x2, r0, r11
	ldsr r11, CPU_REG_FXSR, 10		      /* initialize FXSR */
#endif

#ifdef CONFIG_IAR_DATA_INIT
/* Prepare arguments before initialize global data with __iar_data_init2 */
  mov Region$$Table$$Base, r6
  mov Region$$Table$$Limit, r7
#endif

/* Jump to z_prep_c */
  jarl _z_prep_c, lp
  mov _exit, lp
  jmp [lp]

  end
