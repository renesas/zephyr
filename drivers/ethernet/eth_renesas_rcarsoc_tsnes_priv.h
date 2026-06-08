/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_RENESAS_ETH_TSNES_PRIV_H_
#define ZEPHYR_DRIVERS_ETHERNET_RENESAS_ETH_TSNES_PRIV_H_

#include "zephyr/spinlock.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <ethernet/eth_stats.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CONFIG_ETH_RENESAS_RCARSOC_TSNES_ENABLE_DMA_CACHE_OPS
#define ETH_RAM_SECTION __attribute__((section(".eth_ram")))
#else
#define ETH_RAM_SECTION __nocache
#endif

/* TSNES Hardware Parameters */
#define ETH_TSNES_MAC_ADDR_LEN   6UL
#define ETH_NUM_MAC_FILTERS      32UL
#define ETH_NUM_TX_QUEUES        8UL
#define ETH_NUM_RX_QUEUES        32UL
#define ETH_HEADER_SIZE          14UL
#define ETH_FCS_LENGTH           4UL
#define ETH_SRC_DST_ADDRESS_SIZE 12UL

/* Descriptor Types */
#define ETH_DESC_FEMPTY_ND 0x30
#define ETH_DESC_FEMPTY    0x40
#define ETH_DESC_FSINGLE   0x80
#define ETH_DESC_FSTART    0x90
#define ETH_DESC_FMID      0xA0
#define ETH_DESC_FEND      0xB0
#define ETH_DESC_LEMPTY    0xC0
#define ETH_DESC_EEMPTY    0xD0
#define ETH_DESC_LINK      0xE0
#define ETH_DESC_EOS       0xF0
#define ETH_DESC_DT_MASK   0xF0

#define ETH_DESC_DIE  0x08
#define ETH_DESC_AXIE 0x04
#define ETH_DESC_DSE  0x02

#define ETH_DESC_RX_EI        0x2000
#define ETH_DESC_FCS_INCLUDED 0x1000

/* Initialization Constants */
#define ETH_TSNES_INIT_FRAME_LENGTH     0x00004000UL
#define ETH_TSNES_GIS_SWRCF_MASK        0x00010000UL
#define ETH_TSNES_SOFTWARE_RESET_ENABLE 0x00000001UL
#define ETH_TSNES_TGC1_CONFIG           0x0000FF05UL
#define ETH_TSNES_TCR1_CONFIG           0x00000000UL
#define ETH_TSNES_TCR2_CONFIG           0x00000000UL
#define ETH_TSNES_TCR3_CONFIG           0x00000000UL
#define ETH_TSNES_TCR4_CONFIG           0x00000000UL
#define ETH_TSNES_RGC_CONFIG            0x88A80000UL
#define ETH_TSNES_RDFCR_CONFIG          0x04000800UL
#define ETH_TSNES_RCFCR_CONFIG          0x00800100UL
#define ETH_TSNES_DIE_CONFIG            0x00010101UL
#define ETH_TSNES_TDIE0_CONFIG          0x000000FFUL
#define ETH_TSNES_RDIE0_CONFIG          0x00000001UL
#define ETH_TSNES_RFEIE0_CONFIG         0x00000001UL
#define ETH_TSNES_TSDIE_CONFIG          0x00000001UL
#define ETH_TSNES_EIE0_CONFIG           0x00000003UL
#define ETH_TSNES_TATLS0_CONFIG         0x00000002UL
#define ETH_TSNES_TATLR_LEARNING        0x80000000UL
#define ETH_TSNES_TATLR_FAILED          0x00000001UL
#define ETH_TSNES_RATLS0_CONFIG         0x0000000CUL
#define ETH_TSNES_RATLR_LEARNING        0x80000000UL
#define ETH_TSNES_RATLR_FAILED          0x00000001UL
#define ETH_TSNES_TRCR0_TRANSMIT_ENABLE 0x00000001UL
#define ETH_TSNES_RATRR_RESET           0x00000001UL
#define ETH_TSNES_TATRR_RESET           0x00000002UL
#define ETH_TSNES_AXIWC_CONFIG          0x00014411UL
#define ETH_TSNES_AXIRC_CONFIG          0x00011144UL
#define ETH_TSNES_TDPC_CONFIG           0x76543210UL
#define ETH_TSNES_TFT_CONFIG            0x00000000UL
#define ETH_TSNES_TFS_CONFIG            0x00200020UL
#define ETH_TSNES_MHD_DISABLE_MODE      0x00000000UL
#define ETH_TSNES_MHD_CONFIG_MODE       0x00000001UL
#define ETH_TSNES_MHD_OPERATION_MODE    0x00000002UL

/* MHD Error Interrupt */
#define ETH_TSNES_TCIE_CONFIG      0x00000400UL
#define ETH_TSNES_TIE2_CONFIG      0xFF010000UL
#define ETH_TSNES_RIE_CONFIG       0x00000010UL
#define ETH_TSNES_INIT_TMS_CONFIG  0x00004000UL
#define ETH_TSNES_INIT_TFS0_CONFIG 0x00000100UL

/* RMAC config */
#define ETHER_TSN_MPIC_PIS_GMII  0x00000002UL
#define ETHER_TSN_MPIC_LSC_2500M 0x0000000CUL
#define ETH_TSNES_MPIC_CONFIG    (ETHER_TSN_MPIC_PIS_GMII | ETHER_TSN_MPIC_LSC_2500M)

#define ETH_TSN_GET_RX_FULL_INT_STAT(queue_idx, val, offset)                                       \
	((uint32_t)(val) & ((1 << (uint32_t)(queue_idx)) << (uint32_t)(offset)))

/* TSNES Structures */
struct eth_tsnes_ext_desc {
	uint16_t info_ds;
	uint8_t info;
	volatile uint8_t die_dt;
	uint32_t Dptr;
	uint32_t Info1[2];
} packed;

struct eth_tsnes_ext_desc_with_ts {
	uint16_t info_ds;
	uint8_t info;
	volatile uint8_t die_dt;
	uint32_t Dptr;
	uint32_t Info1[2];
	uint32_t tsns;
	uint32_t tss;
} __packed;

/* Register structs */
typedef struct {
	uint32_t TBFCRi;
	uint32_t TBFVRi;
	uint32_t Reserved0[2];
} eth_tsnes_tbf_reg_t;

typedef struct {
	uint32_t FBFCRi;
	uint32_t FBFV0Ri;
	uint32_t FBFV1Ri;
	uint32_t Reserved0;
} eth_tsnes_fbf_reg_t;

typedef struct {
	uint32_t CFCRi;
	uint32_t SFN0Ri;
	uint32_t SFN1Ri;
	uint32_t Reserved0;
} eth_tsnes_cfsf_reg_t;

typedef struct {
	uint32_t MCC;
	uint32_t Reserved0[3];
	uint32_t MPSM;
	uint32_t MPIC;
	uint32_t Reserved1[2];
	uint32_t MTFFC;
	uint32_t MTPFC;
	uint32_t Reserved2[6];
	uint32_t MTATC[16];
	uint32_t MRGC;
	uint32_t MRMAC0;
	uint32_t MRMAC1;
	uint32_t MRAFC;
	uint32_t MRSCE;
	uint32_t MRSCP;
	uint32_t MRSCC;
	uint32_t MRFSCE;
	uint32_t MRFSCP;
	uint32_t MTRC;
	uint32_t Reserved3[22];
	uint32_t MPFC[16];
	uint32_t Reserved4[128];
	uint32_t MLVC;
	uint32_t Reserved5[3];
	uint32_t MEEEC;
	uint32_t Reserved6[3];
	uint32_t MLBC;
	uint32_t Reserved13[3];
	uint32_t MTIFGC;
	uint32_t MIRFC;
	uint32_t Reserved7[34];
	uint32_t MGMR;
	uint32_t Reserved8[3];
	uint32_t MMPFTCT;
	uint32_t MAPFTCT;
	uint32_t MPFRCT;
	uint32_t MFCICT;
	uint32_t MEEECT;
	uint32_t Reserved9[55];
	uint32_t MEIS;
	uint32_t MEIE;
	uint32_t MEID;
	uint32_t Reserved10;
	uint32_t MMIS0;
	uint32_t MMIE0;
	uint32_t MMID0;
	uint32_t Reserved11;
	uint32_t MMIS1;
	uint32_t MMIE1;
	uint32_t MMID1;
	uint32_t Reserved12;
	uint32_t MMIS2;
	uint32_t MMIE2;
	uint32_t MMID2;
} eth_tsnes_rmac_reg_t;

typedef struct {
	eth_tsnes_tbf_reg_t stTBF[64];
	eth_tsnes_fbf_reg_t stFBF[64];
	eth_tsnes_cfsf_reg_t stCFSF[64];
	uint32_t VTCCR;
	uint32_t Reserved0[3];
	uint32_t FMSCR;
	uint32_t Reserved1[59];
	uint32_t FVLSR;
	uint32_t Reserved2[3];
	uint32_t MVCR;
	uint32_t MV0R;
	uint32_t MV1R;
	uint32_t Reserved3[57];
	uint32_t INTSR;
	uint32_t INTER;
	uint32_t INTDR;
	uint32_t Reserved4[125];
	uint32_t RXBCE;
	uint32_t RXBCP;
	uint32_t RGFCE;
	uint32_t RGFCP;
	uint32_t RBFC;
	uint32_t RMFC;
	uint32_t RVFC;
	uint32_t RPEFC;
	uint32_t RNEFC;
	uint32_t RFMEFC;
	uint32_t RFFMEFC;
	uint32_t RCFCEFC;
	uint32_t RFCEFC;
	uint32_t RRCFEFC;
	uint32_t RRXFEFC;
	uint32_t RUEFC;
	uint32_t ROEFC;
	uint32_t RBOEC;
	uint32_t Reserved5[46];
	uint32_t TXBCE;
	uint32_t TXBCP;
	uint32_t TGFCE;
	uint32_t TGFCP;
	uint32_t TBFC;
	uint32_t TMFC;
	uint32_t TVFC;
	uint32_t TEFC;
	uint32_t Reserved6[440];
	eth_tsnes_rmac_reg_t stRMAC;
} eth_tsnes_rmacs_reg_t;

typedef struct {
	uint32_t TSAs;
	uint32_t TSSs;
} eth_tsnes_tsd_reg_t;

typedef struct {
	uint32_t RIDASMi;
	uint32_t RIDASAMi;
	uint32_t RIDACAMi;
	uint32_t Reserved0;
} eth_tsnes_rid_reg_t;

typedef struct {
	uint32_t TCEISi;
	uint32_t TCEIEi;
	uint32_t TCEIDi;
	uint32_t Reserved0;
} eth_tsnes_tce_reg_t;

typedef struct {
	uint32_t RFSEISi;
	uint32_t RFSEIEi;
	uint32_t RFSEIDi;
	uint32_t Reserved0;
} eth_tsnes_rfs_reg_t;

typedef struct {
	uint32_t RFEISi;
	uint32_t RFEIEi;
	uint32_t RFEIDi;
	uint32_t Reserved0;
} eth_tsnes_rfe_reg_t;

typedef struct {
	uint32_t RCEISi;
	uint32_t RCEIEi;
	uint32_t RCEIDi;
	uint32_t Reserved0;
} eth_tsnes_rce_reg_t;

typedef struct {
	uint32_t TDISi;
	uint32_t TDIEi;
	uint32_t TDIDi;
	uint32_t Reserved0;
} eth_tsnes_tdi_reg_t;

typedef struct {
	uint32_t RDISi;
	uint32_t RDIEi;
	uint32_t RDIDi;
	uint32_t Reserved0;
} eth_tsnes_rdi_reg_t;

typedef struct {
	uint32_t AXIWC;
	uint32_t AXIRC;
	uint32_t Reserved0[2];
	uint32_t TDPC[32];
	uint32_t TFT;
	uint32_t Reserved1[3];
	uint32_t TATLS0;
	uint32_t TATLS1;
	uint32_t TATLR;
	uint32_t Reserved2;
	uint32_t RATLS0;
	uint32_t RATLS1;
	uint32_t RATLR;
	uint32_t Reserved3;
	eth_tsnes_tsd_reg_t stTSD[16];
	uint32_t TRCR[8];
	uint32_t Reserved4[8];
	uint32_t RIDAUAS[8];
	uint32_t Reserved5[24];
	uint32_t RR;
	uint32_t Reserved6[3];
	uint32_t TATS;
	uint32_t TATSR0;
	uint32_t TATSR1;
	uint32_t TATSR2;
	uint32_t RATS;
	uint32_t RATSR0;
	uint32_t RATSR1;
	uint32_t RATSR2;
	uint32_t Reserved7[4];
	eth_tsnes_rid_reg_t stRID[8];
	uint32_t Reserved8[16];
	uint32_t EIS0;
	uint32_t EIE0;
	uint32_t EID0;
	uint32_t Reserved9;
	uint32_t EIS1;
	uint32_t EIE1;
	uint32_t EID1;
	uint32_t Reserved10[9];
	eth_tsnes_tce_reg_t stTXDCEI[8];
	uint32_t Reserved11[64];
	eth_tsnes_rfs_reg_t stRXFSEI[8];
	eth_tsnes_rfe_reg_t stRXDFEI[8];
	eth_tsnes_rce_reg_t stRXDCEI[8];
	uint32_t RIDAOIS;
	uint32_t RIDAOIE;
	uint32_t RIDAOID;
	uint32_t Reserved12[29];
	uint32_t TSFEIS;
	uint32_t TSFEIE;
	uint32_t TSFEID;
	uint32_t Reserved13;
	uint32_t TSCEIS;
	uint32_t TSCEIE;
	uint32_t TSCEID;
	uint32_t Reserved14[265];
	uint32_t DIS;
	uint32_t DIE;
	uint32_t DID;
	uint32_t Reserved15;
	eth_tsnes_tdi_reg_t stTXDI[8];
	eth_tsnes_rdi_reg_t stRXDI[8];
	uint32_t TSDIS;
	uint32_t TSDIE;
	uint32_t TSDID;
} eth_tsnes_axibmi_reg_t;

typedef struct {
	uint32_t QSTMACUs;
	uint32_t QSTMACDs;
	uint32_t QSTMAMUs;
	uint32_t QSTMAMDs;
	uint32_t QSFTVLs;
	uint32_t QSFTVLMs;
	uint32_t QSFTMSDs;
	uint32_t QSFTGMIs;
} eth_tsnes_qcis_reg_t;

typedef struct {
	uint32_t OCR;
	uint32_t OSR;
	uint32_t SWR;
	uint32_t SIS;
	uint32_t GIS;
	uint32_t GIE;
	uint32_t GID;
	uint32_t Reserved0;
	uint32_t TIS1;
	uint32_t TIE1;
	uint32_t TID1;
	uint32_t Reserved1;
	uint32_t TIS2;
	uint32_t TIE2;
	uint32_t TID2;
	uint32_t Reserved2;
	uint32_t RIS;
	uint32_t RIE;
	uint32_t RID;
	uint32_t Reserved3;
	uint32_t TGC1;
	uint32_t TGC2;
	uint32_t Reserved4[2];
	uint32_t TFS[4];
	uint32_t TCF[4];
	uint32_t TCR1;
	uint32_t TCR2;
	uint32_t TCR3;
	uint32_t TCR4;
	uint32_t TMS[8];
	uint32_t TSR1;
	uint32_t TSR2;
	uint32_t TSR3;
	uint32_t TSR4;
	uint32_t TSR5;
	uint32_t Reserved5[3];
	uint32_t RGC;
	uint32_t RDFCR;
	uint32_t RCFCR;
	uint32_t REFCNCR;
	uint32_t RSR1;
	uint32_t RSR2;
	uint32_t RSR3;
	uint32_t Reserved6[61];
	uint32_t TCIS;
	uint32_t TCIE;
	uint32_t TCID;
	uint32_t Reserved7;
	uint32_t TPTPC;
	uint32_t TTML;
	uint32_t TTJ;
	uint32_t Reserved8;
	uint32_t TCC;
	uint32_t TCS;
	uint32_t Reserved9;
	uint32_t TGS;
	uint32_t TACST0;
	uint32_t TACST1;
	uint32_t TACST2;
	uint32_t Reserved10;
	uint32_t TALIT0;
	uint32_t TALIT1;
	uint32_t TALIT2;
	uint32_t Reserved11;
	uint32_t TAEN[2];
	uint32_t Reserved12[2];
	uint32_t TASFE;
	uint32_t Reserved13[3];
	uint32_t TACLL0;
	uint32_t TACLL1;
	uint32_t TACLL2;
	uint32_t Reserved14;
	uint32_t CACC;
	uint32_t CCS;
	uint32_t Reserved15[2];
	uint32_t CAIV[8];
	uint32_t CAUL[8];
	uint32_t Reserved16[20];
	uint32_t TOCST0;
	uint32_t TOCST1;
	uint32_t TOCST2;
	uint32_t Reserved17;
	uint32_t TOLIT0;
	uint32_t TOLIT1;
	uint32_t TOLIT2;
	uint32_t Reserved18;
	uint32_t TOEN0;
	uint32_t TOEN1;
	uint32_t Reserved19[2];
	uint32_t TOSFE;
	uint32_t Reserved20[3];
	uint32_t TCLR0;
	uint32_t TCLR1;
	uint32_t TCLR2;
	uint32_t Reserved21;
	uint32_t TSMS;
	uint32_t Reserved22[3];
	uint32_t COCC;
	uint32_t Reserved23[19];
	uint32_t COIV[8];
	uint32_t COUL[8];
	uint32_t Reserved24[4];
	eth_tsnes_qcis_reg_t stQCIS[16];
	uint32_t QSFTLS;
	uint32_t QSFTLIS;
	uint32_t QSFTLIE;
	uint32_t QSFTLID;
	uint32_t QSMSMC;
	uint32_t QSGTMC;
	uint32_t QSEIS;
	uint32_t QSEIE;
	uint32_t QSEID;
	uint32_t Reserved25[3];
	uint32_t QGACST0;
	uint32_t QGACST1;
	uint32_t QGACST2;
	uint32_t QGALIT0;
	uint32_t QGALIT1;
	uint32_t QGALIT2;
	uint32_t QGAEN0;
	uint32_t QGAEN1;
	uint32_t QGIGS;
	uint32_t QGJC;
	uint32_t QGTL;
	uint32_t QGSE;
	uint32_t QGCC;
	uint32_t QGATL0;
	uint32_t QGATL1;
	uint32_t QGATL2;
	uint32_t QGOCST0;
	uint32_t QGOCST1;
	uint32_t QGOCST2;
	uint32_t QGOLIT0;
	uint32_t QGOLIT1;
	uint32_t QGOLIT2;
	uint32_t QGOEN0;
	uint32_t QGOEN1;
	uint32_t QGTR0;
	uint32_t QGTR1;
	uint32_t QGTR2;
	uint32_t QGFSMS;
	uint32_t Reserved26[16];
	uint32_t QTMIS;
	uint32_t QTMIE;
	uint32_t QTMID;
	uint32_t QTEIS;
	uint32_t QTEIE;
	uint32_t QTEID;
	uint32_t Reserved27[2];
	uint32_t QMEC;
	uint32_t QMMC;
	uint32_t QRFDC;
	uint32_t QYFDC;
	uint32_t QVTCMC[16];
	uint32_t QMCBSC[16];
	uint32_t QMCIRC[16];
	uint32_t QMEBSC[16];
	uint32_t QMEIRC[16];
	uint32_t QMCFC;
	uint32_t Reserved28[3];
	uint32_t QMEIS;
	uint32_t QMEIE;
	uint32_t QMEID;
	uint32_t Reserved29;
	uint32_t QSMFC[16];
	uint32_t QMSPPC[16];
	uint32_t QMSRPC[16];
	uint32_t QGPPC[8];
	uint32_t QGRPC[8];
	uint32_t QMDPC[16];
	uint32_t QMGPC[16];
	uint32_t QMYPC[16];
	uint32_t QMRPC[16];
	uint32_t MQSTMACU;
	uint32_t MQSTMACD;
	uint32_t MQSTMAMU;
	uint32_t MQSTMAMD;
	uint32_t MQSFTVL;
	uint32_t MQSFTVLM;
	uint32_t MQSFTMSD;
	uint32_t MQSFTGMI;
	uint32_t Reserved30[92];
	uint32_t DCR;
	uint32_t Reserved31[7];
	uint32_t TDRRR;
	uint32_t TDRRD;
	uint32_t TCRRR;
	uint32_t TCRRD;
	uint32_t TCD;
	uint32_t Reserved32[3];
	uint32_t RDRRR;
	uint32_t RDRRD;
	uint32_t RCRRR;
	uint32_t RCRRD;
	uint32_t VRR;
} eth_tsnes_mhd_reg_t;

typedef struct {
	eth_tsnes_axibmi_reg_t stAXIBMI;
	uint32_t Reserved0[249];
	eth_tsnes_mhd_reg_t stMHD;
	uint32_t Reserved1[235];
	eth_tsnes_rmacs_reg_t stRMACSys;
} eth_tsnes_reg_t;

/* Queue management */
struct eth_tsnes_queue {
	void *desc_ring;
	struct eth_tsnes_buf_mgmt *buf_nodes;
	uint32_t desc_count;
	uint32_t head_idx;
	uint32_t tail_idx;
	bool rxfull_pending;

	struct {
		uint32_t rx_packets;
		uint32_t rx_bytes;
		uint32_t rx_errors;
		uint32_t rx_dropped;
		uint32_t tx_packets;
		uint32_t tx_bytes;
	} stats;
};

/* Zephyr Driver Configuration Structure */
struct eth_tsnes_config {
	volatile eth_tsnes_reg_t *tsnes_base;
	/** pinctrl configs */
	const struct pinctrl_dev_config *pcfg;

	void (*config_irq)(const struct device *dev);

	/* MAC address */
	uint8_t mac_addr[ETH_TSNES_MAC_ADDR_LEN];

	/* Queue configuration (per-instance, from devicetree) */
	uint8_t num_rx_queues;
	uint8_t num_tx_queues;
	uint16_t rx_queue_size;
	uint16_t tx_queue_size;

	/* Queue memory pointers */
	void *rx_desc_rings[ETH_NUM_RX_QUEUES];
	void *tx_desc_rings[ETH_NUM_TX_QUEUES];
	struct eth_tsnes_buf_mgmt *rx_buf_nodes[ETH_NUM_RX_QUEUES];
	struct eth_tsnes_buf_mgmt *tx_buf_nodes[ETH_NUM_TX_QUEUES];
	uint8_t *buf_pool;
	struct eth_tsnes_ext_desc *init_desc_chain;
	uint8_t *init_dummy_buf;

	/* R-Switch Integration */
	const struct device *rswitch_dev;
	uint32_t channel;
	uint32_t rswitch_if;
	uint32_t rswitch_fwd_ports;
};

/* Zephyr Driver Runtime Data Structure */
struct eth_tsnes_data {
	struct net_if *iface;
	const struct device *dev;

	/* Synchronization */
	struct k_sem rx_sem;
	struct k_spinlock lock;

	/* Buffer management */
	uint8_t *buf_pool;
	uint32_t buf_pool_size;
	uint32_t buf_pool_used;
	uint32_t max_frame_size;

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	/* Networking Stats */
	struct net_stats_eth stats;
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
	/* State */
	bool link_up;
	bool is_started;
	bool initialized;

	/* Queue management */
	struct eth_tsnes_queue rx_queues[ETH_NUM_RX_QUEUES];
	struct eth_tsnes_queue tx_queues[ETH_NUM_TX_QUEUES];

	uint32_t num_rx_queues;
	uint32_t num_tx_queues;

	struct k_work process_work;
	uint32_t pending_irq_status;
	uint32_t pending_rxfull_status;

	/* Synchronization and threading for RX moved to end to prevent stack overflow corruption */
	struct k_thread rx_thread;

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_ETH_RENESAS_RCARSOC_TSNES_RX_THREAD_STACK);
};

struct eth_tsnes_buf_mgmt {
	uintptr_t buf_addr;
	uint32_t buf_size;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_ETHERNET_RENESAS_ETH_TSNES_PRIV_H_ */
