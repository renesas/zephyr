/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_R8A78000_R52_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_R8A78000_R52_H_

/*
 * ============================================================
 * DMARS (DMA Request Source IDs)
 * ============================================================
 */

/* ---------------- RT-DMAC (WCRC blocks) ---------------- */
/**
 * unit: 0 -> 10
 * channel: 0 or 1 (crc or kcrc)
 * type: 0 -> 3 (res_rx, in_tx, out_rx, res_multi)
 * Valid usage:
 * DMA_RT_WCRC_DMARS(0, 1, 2) => wcrc0_kcrc_out_rx = 0x1e
 */
#define DMA_RT_WCRC_DMARS(unit, channel, type)                                                     \
	((((unit) + 1) * 0x10 + 0x8) + ((channel) * 0x4) + (type))

/* ---------------- SYS-DMAC ---------------- */
#define DMA_SYS_TPU_DMARS 0x17

/**
 * channel: 0 -> 3
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_SYS_HSCIF_DMARS(1, 0) => hscif1_rx = 0x32
 */
#define DMA_SYS_HSCIF_DMARS(channel, tx) (((channel) * 2) + (tx) + 0x30)

/**
 * channel: 4 -> 7
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_SYS_MSI_DMARS(6, 1) => msi6_tx = 0x4d
 */
#define DMA_SYS_MSI_DMARS(channel, tx) (((channel) * 2) + (tx) + 0x40)

/**
 * Channel: 1 -> 8
 * slave: 0 or 1 (master or slave)
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_SYS_I2C_DMARS(5, 0, 1) => i2c5_mst_tx = 0x9b
 */
#define DMA_SYS_I2C_DMARS(channel, slave, tx)                                                      \
	((channel) < 8) ? (((slave) * 0x10) + ((channel) * 0x2) + (tx) + 0x90)                     \
			: (((slave) * 0x2) + (tx) + 0xB0)

/**
 * unit: 0 -> 7
 * channel: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_SYS_DRI_DMARS(4, 0) => dri40 = 0xF0
 */
#define DMA_SYS_DRI_DMARS(unit, channel) (((unit) * 0x4) + ((channel) * 0x2) + 0xE0)

/* ---------------- SCP-DMAC ---------------- */
/**
 * slave: 0 or 1 (master or slave)
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_SCP_I2C0_DMARS(0, 1) => i2c0_mst_tx = 0x31
 */
#define DMA_SCP_I2C0_DMARS(slave, tx) (((slave) * 2) + (tx) + 0x30)

/**
 * unit: 0 -> 3
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_SCP_MSI_DMARS(2, 0) => msi2_rx = 0x44
 */
#define DMA_SCP_MSI_DMARS(unit, tx) (((unit) * 2) + (tx) + 0x40)

/**
 * unit: 0 -> 1
 * type: 0 or 1 (rf or cf)
 * channel: 0 -> 7
 * Valid usage:
 * DMA_SCP_CANFD_DMARS(0, 0, 3) => canfd0_rf3 = 0x76
 */
#define DMA_SCP_CANFD_DMARS(unit, type, channel)                                                   \
	(((unit) * 0x50) + ((type) * 0x10) + ((channel) * 0x2) + 0x70)

/* ---------------- AUDIO-DMAC ---------------- */
/**
 * src_num from 0 to 9
 * rx is 0 or 1 (ip or op, HWUM spec)
 * Valid usage:
 * DMA_AUDIO_SCU_SRC_DMARS(8, 0) => scu_srcip8 = 3
 */
#define DMA_AUDIO_SCU_SRC_DMARS(src_num, rx) (2 * (9 - (src_num)) + (rx) + 0x1)

/**
 * channel: 0 -> 9
 * bufid: 0 -> 7
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_AUDIO_SSI_ID_DMARS(9, 5, 1) => ssi_ssid95_tx
 * DMA_AUDIO_SSI_ID_DMARS(7, 0, 0) => ssi_ssid7_rx (ssid 5 to 8 only have 1 bufid)
 * DMA_AUDIO_SSI_ID_DMARS(2, 4, 0) => ssi_ssid24_rx
 */
#define DMA_AUDIO_SSI_ID_DMARS(channel, bufid, tx)                                                 \
	((channel) == 9)                                                                           \
		? ((9 - (channel)) * 0x10 + (7 - (bufid)) * 2 + (tx) + 0x16)                       \
		: (((channel) <= 4) ? ((4 - (channel)) * 0x10 + (7 - (bufid)) * 2 + (tx) + 0x2E)   \
				    : ((8 - (channel)) * 2 + 0x26 + (tx)))

/**
 * channel: 0 -> 9
 * tx: 0 or 1 (rx or tx)
 * Valid usage:
 * DMA_AUDIO_SSI_INDD_DMARS(3, 1) => ssi_ssiindd3_tx = 0x8B
 */
#define DMA_AUDIO_SSI_INDD_DMARS(channel, tx) ((9 - (channel)) * 2 + (tx) + 0x7E)

/**
 * channel: 0 or 1
 * Valid usage:
 * DMA_AUDIO_SCU_CMD_DMARS(1) => scu_cmdop1 = 0x92
 */
#define DMA_AUDIO_SCU_CMD_DMARS(channel) ((1 - (channel)) * 2 + 0x92)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_R8A78000_R52_H_ */
