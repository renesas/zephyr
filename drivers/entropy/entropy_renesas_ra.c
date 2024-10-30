/*
 * Copyright (c) 2024 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <soc.h>
#include <zephyr/drivers/entropy.h>

#include "hw_sce_trng_private.h"
#include "hw_sce_private.h"

static int entropy_renesas_ra_get_entropy(const struct device *dev, uint8_t *buf, uint16_t len)
{
	ARG_UNUSED(dev);

	if (buf == NULL) {
		return -EINVAL;
	}

	while (len > 0) {
		uint32_t n[4];
		const uint32_t to_copy = MIN(sizeof(n), len);

		if (FSP_SUCCESS != HW_SCE_RNG_Read(n)) {
			return -ENODATA;
		}
		memcpy(buf, n, to_copy);
		buf += to_copy;
		len -= to_copy;
	}

	return 0;
}

static const struct entropy_driver_api entropy_renesas_ra_api = {
	.get_entropy = entropy_renesas_ra_get_entropy,
};

static int entropy_renesas_ra_init(const struct device *dev)
{
	HW_SCE_McuSpecificInit();
	return 0;
}

#define RENESAS_RA_ENTROPY_DEVICE_DEFINE(nodeid)                                                   \
	DEVICE_DT_DEFINE(nodeid, entropy_renesas_ra_init, NULL, NULL, NULL, PRE_KERNEL_1,          \
			 CONFIG_ENTROPY_INIT_PRIORITY, &entropy_renesas_ra_api)

#define RENESAS_RA_RSIP_E51A_RNG_DEVICE_INIT_IF_OKAY(n)                                            \
	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(DT_INST(n, renesas_ra_rsip_e51a_trng)),		   \
		    (RENESAS_RA_ENTROPY_DEVICE_DEFINE(DT_INST(n, renesas_ra_rsip_e51a_trng))), ())

#define RENESAS_RA_SCE7_RNG_DEVICE_INIT_IF_OKAY(n)                                                 \
	COND_CODE_1(DT_NODE_HAS_STATUS_OKAY(DT_INST(n, renesas_ra_sce7_rng)),			   \
		    (RENESAS_RA_ENTROPY_DEVICE_DEFINE(DT_INST(n, renesas_ra_sce7_rng))), ())

RENESAS_RA_RSIP_E51A_RNG_DEVICE_INIT_IF_OKAY(0)
RENESAS_RA_SCE7_RNG_DEVICE_INIT_IF_OKAY(0)
