/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * ============================================================
 * Register Offsets
 * ============================================================
 */
#define RCAR_DMA_OR      0x60
#define RCAR_DMA_RATE_RD 0xF4
#define RCAR_DMA_RATE_WR 0xF8
#define RCAR_DMA_CHCR    0x0C
#define RCAR_DMA_CHCRB   0x1C
#define RCAR_DMA_TSR     0x28
#define RCAR_DMA_TSRB    0x38
#define RCAR_DMA_RS      0x40
#define RCAR_DMA_DPBASE  0x50
#define RCAR_DMA_EVTCR   0x58
#define RCAR_DMA_EVTCNT  0x5C
#define RCAR_DMA_CHCLR   0x100
#define RCAR_DMA_ISTA    0x110

#define RCAR_CHANNEL_OFFSET 0x1000

/*
 * ============================================================
 * Global Control MACROs
 * ============================================================
 */
#define RCAR_DMA_MASTER_ENABLE BIT(0)
#define RCAR_DMA_DESCRRIPTOR_MAXIMUM_TRANSFER_SIZE (0xFFFFFFU)

/*
 * ============================================================
 * CHCR Bitfields
 * ============================================================
 */
#define CHCR_CAE_POS  31
#define CHCR_CAE_MASK 0x1

#define CHCR_CAIE_POS  30
#define CHCR_CAIE_MASK 0x1

#define CHCR_DPM_POS  28
#define CHCR_DPM_MASK 0x3

#define CHCR_RPT_POS  24
#define CHCR_RPT_MASK 0xF

#define CHCR_WAIT_POS  23
#define CHCR_WAIT_MASK 0x1

#define CHCR_DPB_POS  22
#define CHCR_DPB_MASK 0x1

#define CHCR_TS23_POS  20
#define CHCR_TS23_MASK 0x3

#define CHCR_DSE_POS  19
#define CHCR_DSE_MASK 0x1

#define CHCR_DSIE_POS  18
#define CHCR_DSIE_MASK 0x1

#define CHCR_DM_POS  14
#define CHCR_DM_MASK 0x3

#define CHCR_SM_POS  12
#define CHCR_SM_MASK 0x3

#define CHCR_RS_POS  8
#define CHCR_RS_MASK 0xF

#define CHCR_TS01_POS  3
#define CHCR_TS01_MASK 0x3

#define CHCR_IE_POS  2
#define CHCR_IE_MASK 0x1

#define CHCR_TE_POS  1
#define CHCR_TE_MASK 0x1

#define CHCR_DE_POS  0
#define CHCR_DE_MASK 0x1

/*
 * ============================================================
 * CHCR Convenience Macros
 * ============================================================
 */
#define RCAR_DMA_CHCR_NORMAL_MODE BIT(28)
#define RCAR_DMA_CHCR_REPEAT_MODE BIT(29)

#define RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_SAR  BIT(27)
#define RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_DAR  BIT(26)
#define RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_TCR  BIT(25)
#define RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_CHCR BIT(24)

#define RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_ALL                                                        \
	(RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_SAR | RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_DAR |               \
	 RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_TCR | RCAR_DMA_CHCR_DESCRIPTOR_UPDATE_CHCR)

#define RCAR_DMA_CHCR_START_FROM_DESC BIT(22)

#define RCAR_DMA_CHCR_CHANNEL_SUSPEND BIT(23)
#define RCAR_DMA_CHCR_CHANNEL_ENABLE  BIT(0)

#define RCAR_DMA_CHCR_AUTOREQUEST BIT(10)
#define RCAR_DMA_CHCR_EXTENDEDREQ BIT(11)

#define RCAR_DMA_CHCR_CHANNEL_CAE_FLAG BIT(31)
#define RCAR_DMA_CHCR_CHANNEL_DSE_FLAG BIT(19)
#define RCAR_DMA_CHCR_CHANNEL_TE_FLAG  BIT(1)

#define RCAR_DMA_CHCR_ADDERR_INT_ENABLE BIT(30)
#define RCAR_DMA_CHCR_TEND_INT_ENABLE   BIT(2)

/*
 * ============================================================
 * CHCRB Bitfields
 * ============================================================
 */
#define CHCRB_DCNT_POS  24
#define CHCRB_DCNT_MASK 0xFF

#define CHCRB_DPTR_POS  16
#define CHCRB_DPTR_MASK 0xFF

#define CHCRB_DRST_POS  15
#define CHCRB_DRST_MASK 0x1

#define CHCRB_DREQOSEN_POS  14
#define CHCRB_DREQOSEN_MASK 0x1

#define CHCRB_DSIEEN_POS  10
#define CHCRB_DSIEEN_MASK 0x1

#define CHCRB_DTS_POS  8
#define CHCRB_DTS_MASK 0x1

#define CHCRB_SLM_POS  4
#define CHCRB_SLM_MASK 0xF

#define CHCRB_PRI_POS  0
#define CHCRB_PRI_MASK 0xF

#define CHCRB_DRST_RESET BIT(15)

/*
 * ============================================================
 * Event / Status
 * ============================================================
 */
#define RCAR_DMA_EVTCR_COUNTER_RESET BIT(7)
#define RCAR_DMA_EVTCNT_COUNTER_INCR BIT(0)

#define RCAR_DMA_ISTA_INTERRUPT_STATUS BIT(0)

#define CHCLR_CLR BIT(0)

/*
 * ============================================================
 * Descriptor Fields
 * ============================================================
 */
#define DESCRIPTOR_CHCR_DRS_POS  16
#define DESCRIPTOR_CHCR_DRS_MASK 0xFF

#define DESCRIPTOR_CHCR_DM_POS  10
#define DESCRIPTOR_CHCR_DM_MASK 0x3

#define DESCRIPTOR_CHCR_SM_POS  8
#define DESCRIPTOR_CHCR_SM_MASK 0x3

#define DESCRIPTOR_CHCR_RS_POS  6
#define DESCRIPTOR_CHCR_RS_MASK 0x3

#define DESCRIPTOR_CHCR_WAIT_POS  5
#define DESCRIPTOR_CHCR_WAIT_MASK 0x1

#define DESCRIPTOR_CHCR_TS_POS  0
#define DESCRIPTOR_CHCR_TS_MASK 0xF

/*
 * ============================================================
 * Descriptor Configuration Values
 * ============================================================
 */
#define DESCRIPTOR_ADDRESS_MODE_FIXED 0
#define DESCRIPTOR_ADDRESS_MODE_INC   1
#define DESCRIPTOR_ADDRESS_MODE_DEC   2

#define DESCRIPTOR_RESOURCE_AUTOREQ  0x1
#define DESCRIPTOR_RESOURCE_EXTENDED 0x2

#define DESCRIPTOR_TRANSFER_1_BYTE  0
#define DESCRIPTOR_TRANSFER_2_BYTE  1
#define DESCRIPTOR_TRANSFER_4_BYTE  2
#define DESCRIPTOR_TRANSFER_8_BYTE  7
#define DESCRIPTOR_TRANSFER_16_BYTE 3
#define DESCRIPTOR_TRANSFER_32_BYTE 4
#define DESCRIPTOR_TRANSFER_64_BYTE 5

/*
 * ============================================================
 * DMAC Instance
 * ============================================================
 */

#define _DMAC_INSTANCE(type, channel) type##channel
#define DMAC_INSTANCE(type, channel)  _DMAC_INSTANCE(type, channel)

#define PUBLIC_DEV_INDEX_OFFSET 4
#define GET_PUBLIC_DEV_INDEX(dev) dev + PUBLIC_DEV_INDEX_OFFSET
/*
 * ============================================================
 * Enums
 * ============================================================
 */
enum dmac_type {
	RT_DMAC = 0,
	SYS_DMAC,
	AUDIO_DMAC,
	SCP_DMAC,
	RTT_DMAC
};

enum channel_state {
	DMA_IDLE,       /* Not configured (free after init or release) */
	DMA_CONFIGURED, /* Configured, ready to transfer */
	DMA_RUNNING,    /* Transfer in progress */
	DMA_SUSPENDED,  /* Transfer paused */
};
