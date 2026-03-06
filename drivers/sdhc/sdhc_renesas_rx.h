/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define SDHI_PRV_ACCESS_TIMEOUT_US       100000U
#define SDHI_PRV_SD_OPTION_DEFAULT       0x40E0U
#define SDHI_PRV_SD_OPTION_WIDTH8_BIT    13
#define SDHI_PRV_BYTES_PER_KILOBYTE      1024
#define SDHI_PRV_SECTOR_COUNT_IN_EXT_CSD 0xFFFU
#define SDHI_TIME_OUT_MAX                0xFFFFFFFF
#define SDHI_PRV_RESPONSE_BIT            0

struct sdmmc_rx_event {
	volatile bool transfer_completed;
	struct k_sem transfer_sem;
};

struct sdmmc_rx_command {
	uint32_t opcode;
	uint32_t arg;
	void *data;
	unsigned int sector_count;
	unsigned int sector_size;
	int timeout_ms;
};
