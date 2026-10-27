/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT renesas_rcar_backup_buffer

#include <zephyr/device.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rcar_backup_buffer, CONFIG_SOC_LOG_LEVEL);

/*
 * MDLCnPKCPROT1 write protection: bits [31:8] hold the key code, bit [0]
 * releases the protection of the registers it guards, MDLCnBKBAPR included.
 */
#define BKB_PROT_WRITE_DISABLE (0xA5A5A501U)
#define BKB_PROT_WRITE_ENABLE  (0xA5A5A500U)

/* MDLCnBKBAPR access permission of the Back-up Buffer */
#define BKB_ACCESS_PERMITTED (0x00000001U)
#define BKB_ACCESS_PROTECTED (0x00000000U)

struct rcar_backup_buffer_config {
	mem_addr_t prot;
	mem_addr_t bkbapr;
};

static int rcar_backup_buffer_init(const struct device *dev)
{
	const struct rcar_backup_buffer_config *config = dev->config;

	/* Permit access to the Back-up Buffer */
	sys_write32(BKB_PROT_WRITE_DISABLE, config->prot);
	sys_write32(BKB_ACCESS_PERMITTED, config->bkbapr);
	sys_write32(BKB_PROT_WRITE_ENABLE, config->prot);

	return 0;
}

#define RCAR_BACKUP_BUFFER_INIT(n)                                                                 \
	static const struct rcar_backup_buffer_config rcar_backup_buffer_cfg_##n = {               \
		.prot = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(n, prot),                             \
		.bkbapr = (mem_addr_t)DT_INST_REG_ADDR_BY_NAME(n, bkbapr),                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, rcar_backup_buffer_init, NULL, NULL, &rcar_backup_buffer_cfg_##n, \
			      PRE_KERNEL_1, CONFIG_RCAR_BACKUP_BUFFER_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(RCAR_BACKUP_BUFFER_INIT)
