/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/toolchain.h>
#include <bsp_api.h>

#include "ospi_verify.h"

#define CM33_IMAGE_NODE    DT_NODELABEL(cm33_ospi_partition)
#define CM33_MANIFEST_NODE DT_NODELABEL(cm33_ospi_manifest_partition)

#define CM33_IMAGE_ADDR     DT_REG_ADDR(CM33_IMAGE_NODE)
#define CM33_IMAGE_MAX_SIZE DT_REG_SIZE(CM33_IMAGE_NODE)

/* Written by scripts/ospi_sign.py ("manifest" command) */
#define CM33_MANIFEST_MAGIC 0x33334D43U /* "CM33" */

/* Key code of CPU1ACTCSR, see R_BSP_SecondaryCoreStart() */
#define CPU1ACTCSR_KEY_CODE 0xA5U

struct cm33_manifest {
	uint32_t magic;
	uint32_t image_size;
	uint8_t sig[OSPI_SIG_LEN];
} __packed;

BUILD_ASSERT(sizeof(struct cm33_manifest) <= DT_REG_SIZE(CM33_MANIFEST_NODE),
	     "Cortex-M33 manifest does not fit its partition");
/* CPU1INITVTOR[6:0] are discarded by the hardware */
BUILD_ASSERT((CM33_IMAGE_ADDR & 0x7FU) == 0U,
	     "Cortex-M33 vector table must be 128-byte aligned");

int cm33_verify_and_start(void)
{
	const volatile struct cm33_manifest *manifest =
		(const volatile struct cm33_manifest *)DT_REG_ADDR(CM33_MANIFEST_NODE);
	const uint32_t *vector_table = (const uint32_t *)CM33_IMAGE_ADDR;
	uint32_t image_size;
	uint32_t reset_handler;
	int ret;

	if (manifest->magic != CM33_MANIFEST_MAGIC) {
		printk("CM33 manifest not found at %p\n", (const void *)manifest);
		return -ENOENT;
	}

	/* Read the size once: the signature check and the bounds check must
	 * use the same value.
	 */
	image_size = manifest->image_size;
	if ((image_size < (2U * sizeof(uint32_t))) || (image_size > CM33_IMAGE_MAX_SIZE)) {
		printk("CM33 image size %u out of range\n", image_size);
		return -EINVAL;
	}

	ret = ospi_mldsa87_verify((const uint8_t *)CM33_IMAGE_ADDR, image_size,
				  (const uint8_t *)manifest->sig);
	if (ret != 0) {
		return ret;
	}

	/* The vector table is part of the signed image; still refuse to start
	 * a reset handler that is not inside it.
	 */
	reset_handler = vector_table[1] & ~1U;
	if ((reset_handler < CM33_IMAGE_ADDR) || (reset_handler >= (CM33_IMAGE_ADDR + image_size))) {
		printk("CM33 reset handler 0x%08x outside the image\n", reset_handler);
		return -EINVAL;
	}

	R_CPU_CTRL->CPU1INITVTOR = CM33_IMAGE_ADDR;

	/* A debugger may already have activated CPU1 with CPU1WAITCR set */
	R_CPU_CTRL->CPU1WAITCR = 0U;

	R_CPU_CTRL->CPU1ACTCSR = (CPU1ACTCSR_KEY_CODE << R_CPU_CTRL_CPU1ACTCSR_KEY_Pos) |
				 R_CPU_CTRL_CPU1ACTCSR_ACTREQ_Msk;

	return 0;
}
