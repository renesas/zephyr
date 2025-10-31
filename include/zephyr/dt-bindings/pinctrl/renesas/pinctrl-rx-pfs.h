/*
 * Copyright (c) 2024-2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_RX_H__
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_RX_H__

#define RX_PORT_NUM_POS  0
#define RX_PORT_NUM_MASK 0x1f

#define RX_PIN_NUM_POS  6
#define RX_PIN_NUM_MASK 0xf

#define RX_PSEL_HIZ_JTAG_SWD      0x00
#define RX_PSEL_RTCIC             0x00
#define RX_PSEL_GPT0              0x01
#define RX_PSEL_GPT1              0x02
#define RX_PSEL_GPT2              0x03
#define RX_PSEL_GPT3              0x04
#define RX_PSEL_TMR               0x05
#define RX_PSEL_RTC               0x06
#define RX_PSEL_CAC               0x07
#define RX_PSEL_POEG              0x08
#define RX_PSEL_ADC               0x09
#define RX_PSEL_SCI_0             0x0A
#define RX_PSEL_SCI_2             0x0A
#define RX_PSEL_SCI_4             0x0A
#define RX_PSEL_SCI_6             0x0A
#define RX_PSEL_SCI_8             0x0A
#define RX_PSEL_SCI_10            0x0A
#define RX_PSEL_SCI_12            0x0A
#define RX_PSEL_SCI_0_CTS_DE      0x0B
#define RX_PSEL_SCI_2_CTS_DE      0x0B
#define RX_PSEL_SCI_4_CTS_DE      0x0B
#define RX_PSEL_SCI_6_CTS_DE      0x0B
#define RX_PSEL_SCI_8_CTS_DE      0x0B
#define RX_PSEL_SCI_10_CTS_DE     0x0B
#define RX_PSEL_SCI_12_CTS_DE     0x0B
#define RX_PSEL_SCI_0_HBS         0x0C
#define RX_PSEL_SCI_2_HBS         0x0C
#define RX_PSEL_SCI_4_HBS         0x0C
#define RX_PSEL_SCI_6_HBS         0x0C
#define RX_PSEL_SCI_8_C_HBS       0x0C
#define RX_PSEL_SCI_10_HBS        0x0C
#define RX_PSEL_SCI_12_HBS        0x0C
#define RX_PSEL_SCI_1             0x0D
#define RX_PSEL_SCI_3             0x0D
#define RX_PSEL_SCI_5             0x0D
#define RX_PSEL_SCI_7             0x0D
#define RX_PSEL_SCI_9             0x0D
#define RX_PSEL_SCI_11            0x0D
#define RX_PSEL_SCI_1_CTS_DE      0x0E
#define RX_PSEL_SCI_3_CTS_DE      0x0E
#define RX_PSEL_SCI_5_CTS_DE      0x0E
#define RX_PSEL_SCI_7_CTS_DE      0x0E
#define RX_PSEL_SCI_9_CTS_DE      0x0E
#define RX_PSEL_SCI_11_CTS_DE     0x0E
#define RX_PSEL_SCI_1_HBS         0x0F
#define RX_PSEL_SCI_3_HBS         0x0F
#define RX_PSEL_SCI_5_HBS         0x0F
#define RX_PSEL_SCI_7_HBS         0x0F
#define RX_PSEL_SCI_9_HBS         0x0F
#define RX_PSEL_SCI_11_HBS        0x0Fs
#define RX_PSEL_SPI               0x10
#define RX_PSEL_I3C               0x11
#define RX_PSEL_CANFD             0x12
#define RX_PSEL_USBFS             0x13
#define RX_PSEL_USBHS             0x14
#define RX_PSEL_DSMIF	          0x15
#define RX_PSEL_PDMIF             0x16
#define RX_PSEL_SSIE              0x17
#define RX_PSEL_PCIF              0x18
#define RX_PSEL_SDHI              0x19
#define RX_PSEL_XSPI0_1A          0x1A
#define RX_PSEL_XSPI0_1B          0x1B
#define RX_PSEL_XSPI1_1A          0x1C
#define RX_PSEL_XSPI1_1B          0x1D
#define RX_PSEL_PDC               0x1C
#define RX_PSEL_CMTW              0x1D
#define RX_PSEL_CLKOUT            0x1F
#define RX_PSEL_ETH_RMAC_GMII_MII 0x20
#define RX_PSEL_ETH_RMAC_RMII     0x21
#define RX_PSEL_ETH_RMAC_RGMII    0x22
#define RX_PSEL_ETH_PTP           0x23
#define RX_PSEL_ETH_ESC0          0x24
#define RX_PSEL_ETH_ESC1          0x25
#define RX_PSEL_EXBUS_SDRAM_0     0x3D
#define RX_PSEL_EXBUS_SDRAM_1     0x3E
#define RX_PSEL_PIO               0x40

#define RX_PSEL_POS  11
#define RX_PSEL_MASK 0x7f

#define RX_MODE_POS  18
#define RX_MODE_MASK 0x1

#define RX_PSEL(psel, port_num, pin_num)                                                           \
	(1 << RX_MODE_POS | psel << RX_PSEL_POS | port_num << RX_PORT_NUM_POS |                    \
	 pin_num << RX_PIN_NUM_POS)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RENESAS_PINCTRL_RX_H__ */
