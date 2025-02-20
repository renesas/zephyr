/*
 * Copyright (c) 2021 KT-Elektronik, Klaucke und Partner GmbH
 * Copyright (c) 2024 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 *
 *
 * Option-Setting Memory for the RX. This region of memory (located in flash)
 * determines the state of the MCU after reset and can not be changed on runtime
 *
 * All registers are set to 0xffffffff by default, which are "safe" settings.
 * Please refer to the Renesas RX Group User's Manual before changing any of
 * the values as some changes can be permanent or lock access to the device.
 *
 * Address range: 0xFE7F5D00 to 0xFE7F5D7F (128 Bytes)
 */

#define __OFS_MDE __attribute__((section(".ofs_mde")))
#define __OFS0 __attribute__((section(".ofs0")))
#define __OFS1 __attribute__((section(".ofs1")))

/* Endian Select Register (MDE) at 0xFE7F5D00
 *
 * b2 to b0: endian select between (0 0 0) for big endian and (1 1 1) for little
 * endian. Set this according to __BYTE_ORDER__ (cf. include\toolchain\gcc.h)
 *
 * b6-b4 (Bank Mode Select) indicate whether the flash is operated in
 * Dual mode (0 0 0) or Linear mode (1 1 1).
 *
 * all other bits are reserved and have to be set to 1
 */
const unsigned long __OFS_MDE __MDEreg = 0xffffffff; /* little */

struct st_ofs0 {
	unsigned long res0: 1;
	unsigned long IWDTSTRT: 1;
	unsigned long IWDTTOPS: 2;
	unsigned long IWDTCKS: 4;
	unsigned long IWDTRPES: 2;
	unsigned long IWDTRPSS: 2;
	unsigned long IWDTRSTIRQS: 1;
	unsigned long res1: 1;
	unsigned long IWDTSLCSTP: 1;
	unsigned long res2: 16;
};

/* Option Function Select Register 0 (OFS0) */
const struct st_ofs0 __OFS0 __OFS0reg = {
	.res0 = 1,
	.IWDTSTRT = CONFIG_IWDT_RENESAS_RX_IWDTSTRT,
#ifdef CONFIG_IWDT_RENESAS_RX_AUTO_START_MODE
	.IWDTTOPS = IWDT_RENESAS_RX_OFS0_IWDTTOPS,
	.IWDTCKS = IWDT_RENESAS_RX_OFS0_IWDTCKS,
	.IWDTRPES = IWDT_RENESAS_RX_OFS0_IWDTRPES,
	.IWDTRPSS = IWDT_RENESAS_RX_OFS0_IWDTRPSS,
	.IWDTRSTIRQS = IWDT_RENESAS_RX_OFS0_IWDTRSTIRQS,
	.res1 = 1,
	.IWDTSLCSTP = IWDT_RENESAS_RX_OFS0_IWDTSLCSTP,
#endif
	.res2 = 0xFFFF
};

/* Option Function Select Register 1 (OFS1) at 0xFE7F5D08 (Voltage detection and
 * HOCO)
 */
const unsigned long __OFS1 __OFS1reg = 0xffffffff;
