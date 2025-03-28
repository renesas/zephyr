/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_MISC_RENESAS_RA_ELC_RA2A1_ELC_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_MISC_RENESAS_RA_ELC_RA2A1_ELC_H_

/* Sources of event signals to be linked to other peripherals or the CPU */
#define RA_ELC_EVENT_NONE                   0x0
#define RA_ELC_EVENT_ICU_IRQ0               0x001
#define RA_ELC_EVENT_ICU_IRQ1               0x002
#define RA_ELC_EVENT_ICU_IRQ2               0x003
#define RA_ELC_EVENT_ICU_IRQ3               0x004
#define RA_ELC_EVENT_ICU_IRQ4               0x005
#define RA_ELC_EVENT_ICU_IRQ5               0x006
#define RA_ELC_EVENT_ICU_IRQ6               0x007
#define RA_ELC_EVENT_ICU_IRQ7               0x008
#define RA_ELC_EVENT_DTC_COMPLETE           0x009
#define RA_ELC_EVENT_DTC_END                0x00A
#define RA_ELC_EVENT_ICU_SNOOZE_CANCEL      0x00B
#define RA_ELC_EVENT_FCU_FRDYI              0x00C
#define RA_ELC_EVENT_LVD_LVD1               0x00D
#define RA_ELC_EVENT_LVD_LVD2               0x00E
#define RA_ELC_EVENT_CGC_MOSC_STOP          0x00F
#define RA_ELC_EVENT_LPM_SNOOZE_REQUEST     0x010
#define RA_ELC_EVENT_AGT0_INT               0x011
#define RA_ELC_EVENT_AGT0_COMPARE_A         0x012
#define RA_ELC_EVENT_AGT0_COMPARE_B         0x013
#define RA_ELC_EVENT_AGT1_INT               0x014
#define RA_ELC_EVENT_AGT1_COMPARE_A         0x015
#define RA_ELC_EVENT_AGT1_COMPARE_B         0x016
#define RA_ELC_EVENT_IWDT_UNDERFLOW         0x017
#define RA_ELC_EVENT_WDT_UNDERFLOW          0x018
#define RA_ELC_EVENT_RTC_ALARM              0x019
#define RA_ELC_EVENT_RTC_PERIOD             0x01A
#define RA_ELC_EVENT_RTC_CARRY              0x01B
#define RA_ELC_EVENT_ADC0_SCAN_END          0x01C
#define RA_ELC_EVENT_ADC0_SCAN_END_B        0x01D
#define RA_ELC_EVENT_ADC0_WINDOW_A          0x01E
#define RA_ELC_EVENT_ADC0_WINDOW_B          0x01F
#define RA_ELC_EVENT_ADC0_COMPARE_MATCH     0x020
#define RA_ELC_EVENT_ADC0_COMPARE_MISMATCH  0x021
#define RA_ELC_EVENT_ACMPHS0_INT            0x022
#define RA_ELC_EVENT_ACMPLP0_INT            0x023
#define RA_ELC_EVENT_ACMPLP1_INT            0x024
#define RA_ELC_EVENT_USBFS_INT              0x025
#define RA_ELC_EVENT_USBFS_RESUME           0x026
#define RA_ELC_EVENT_IIC0_RXI               0x027
#define RA_ELC_EVENT_IIC0_TXI               0x028
#define RA_ELC_EVENT_IIC0_TEI               0x029
#define RA_ELC_EVENT_IIC0_ERI               0x02A
#define RA_ELC_EVENT_IIC0_WUI               0x02B
#define RA_ELC_EVENT_IIC1_RXI               0x02C
#define RA_ELC_EVENT_IIC1_TXI               0x02D
#define RA_ELC_EVENT_IIC1_TEI               0x02E
#define RA_ELC_EVENT_IIC1_ERI               0x02F
#define RA_ELC_EVENT_CTSU_WRITE             0x030
#define RA_ELC_EVENT_CTSU_READ              0x031
#define RA_ELC_EVENT_CTSU_END               0x032
#define RA_ELC_EVENT_KEY_INT                0x033
#define RA_ELC_EVENT_DOC_INT                0x034
#define RA_ELC_EVENT_CAC_FREQUENCY_ERROR    0x035
#define RA_ELC_EVENT_CAC_MEASUREMENT_END    0x036
#define RA_ELC_EVENT_CAC_OVERFLOW           0x037
#define RA_ELC_EVENT_CAN0_ERROR             0x038
#define RA_ELC_EVENT_CAN0_FIFO_RX           0x039
#define RA_ELC_EVENT_CAN0_FIFO_TX           0x03A
#define RA_ELC_EVENT_CAN0_MAILBOX_RX        0x03B
#define RA_ELC_EVENT_CAN0_MAILBOX_TX        0x03C
#define RA_ELC_EVENT_IOPORT_EVENT_1         0x03D
#define RA_ELC_EVENT_IOPORT_EVENT_2         0x03E
#define RA_ELC_EVENT_ELC_SOFTWARE_EVENT_0   0x03F
#define RA_ELC_EVENT_ELC_SOFTWARE_EVENT_1   0x040
#define RA_ELC_EVENT_POEG0_EVENT            0x041
#define RA_ELC_EVENT_POEG1_EVENT            0x042
#define RA_ELC_EVENT_SDADC0_ADI             0x043
#define RA_ELC_EVENT_SDADC0_SCANEND         0x044
#define RA_ELC_EVENT_SDADC0_CALIEND         0x045
#define RA_ELC_EVENT_GPT0_CAPTURE_COMPARE_A 0x046
#define RA_ELC_EVENT_GPT0_CAPTURE_COMPARE_B 0x047
#define RA_ELC_EVENT_GPT0_COMPARE_C         0x048
#define RA_ELC_EVENT_GPT0_COMPARE_D         0x049
#define RA_ELC_EVENT_GPT0_COUNTER_OVERFLOW  0x04A
#define RA_ELC_EVENT_GPT0_COUNTER_UNDERFLOW 0x04B
#define RA_ELC_EVENT_GPT1_CAPTURE_COMPARE_A 0x04C
#define RA_ELC_EVENT_GPT1_CAPTURE_COMPARE_B 0x04D
#define RA_ELC_EVENT_GPT1_COMPARE_C         0x04E
#define RA_ELC_EVENT_GPT1_COMPARE_D         0x04F
#define RA_ELC_EVENT_GPT1_COUNTER_OVERFLOW  0x050
#define RA_ELC_EVENT_GPT1_COUNTER_UNDERFLOW 0x051
#define RA_ELC_EVENT_GPT2_CAPTURE_COMPARE_A 0x052
#define RA_ELC_EVENT_GPT2_CAPTURE_COMPARE_B 0x053
#define RA_ELC_EVENT_GPT2_COMPARE_C         0x054
#define RA_ELC_EVENT_GPT2_COMPARE_D         0x055
#define RA_ELC_EVENT_GPT2_COUNTER_OVERFLOW  0x056
#define RA_ELC_EVENT_GPT2_COUNTER_UNDERFLOW 0x057
#define RA_ELC_EVENT_GPT3_CAPTURE_COMPARE_A 0x058
#define RA_ELC_EVENT_GPT3_CAPTURE_COMPARE_B 0x059
#define RA_ELC_EVENT_GPT3_COMPARE_C         0x05A
#define RA_ELC_EVENT_GPT3_COMPARE_D         0x05B
#define RA_ELC_EVENT_GPT3_COUNTER_OVERFLOW  0x05C
#define RA_ELC_EVENT_GPT3_COUNTER_UNDERFLOW 0x05D
#define RA_ELC_EVENT_GPT4_CAPTURE_COMPARE_A 0x05E
#define RA_ELC_EVENT_GPT4_CAPTURE_COMPARE_B 0x05F
#define RA_ELC_EVENT_GPT4_COMPARE_C         0x060
#define RA_ELC_EVENT_GPT4_COMPARE_D         0x061
#define RA_ELC_EVENT_GPT4_COUNTER_OVERFLOW  0x062
#define RA_ELC_EVENT_GPT4_COUNTER_UNDERFLOW 0x063
#define RA_ELC_EVENT_GPT5_CAPTURE_COMPARE_A 0x064
#define RA_ELC_EVENT_GPT5_CAPTURE_COMPARE_B 0x065
#define RA_ELC_EVENT_GPT5_COMPARE_C         0x066
#define RA_ELC_EVENT_GPT5_COMPARE_D         0x067
#define RA_ELC_EVENT_GPT5_COUNTER_OVERFLOW  0x068
#define RA_ELC_EVENT_GPT5_COUNTER_UNDERFLOW 0x069
#define RA_ELC_EVENT_GPT6_CAPTURE_COMPARE_A 0x06A
#define RA_ELC_EVENT_GPT6_CAPTURE_COMPARE_B 0x06B
#define RA_ELC_EVENT_GPT6_COMPARE_C         0x06C
#define RA_ELC_EVENT_GPT6_COMPARE_D         0x06D
#define RA_ELC_EVENT_GPT6_COUNTER_OVERFLOW  0x06E
#define RA_ELC_EVENT_GPT6_COUNTER_UNDERFLOW 0x06F
#define RA_ELC_EVENT_OPS_UVW_EDGE           0x070
#define RA_ELC_EVENT_SCI0_RXI               0x071
#define RA_ELC_EVENT_SCI0_TXI               0x072
#define RA_ELC_EVENT_SCI0_TEI               0x073
#define RA_ELC_EVENT_SCI0_ERI               0x074
#define RA_ELC_EVENT_SCI0_AM                0x075
#define RA_ELC_EVENT_SCI0_RXI_OR_ERI        0x076
#define RA_ELC_EVENT_SCI1_RXI               0x077
#define RA_ELC_EVENT_SCI1_TXI               0x078
#define RA_ELC_EVENT_SCI1_TEI               0x079
#define RA_ELC_EVENT_SCI1_ERI               0x07A
#define RA_ELC_EVENT_SCI1_AM                0x07B
#define RA_ELC_EVENT_SCI9_RXI               0x07C
#define RA_ELC_EVENT_SCI9_TXI               0x07D
#define RA_ELC_EVENT_SCI9_TEI               0x07E
#define RA_ELC_EVENT_SCI9_ERI               0x07F
#define RA_ELC_EVENT_SCI9_AM                0x080
#define RA_ELC_EVENT_SPI0_RXI               0x081
#define RA_ELC_EVENT_SPI0_TXI               0x082
#define RA_ELC_EVENT_SPI0_IDLE              0x083
#define RA_ELC_EVENT_SPI0_ERI               0x084
#define RA_ELC_EVENT_SPI0_TEI               0x085
#define RA_ELC_EVENT_SPI1_RXI               0x086
#define RA_ELC_EVENT_SPI1_TXI               0x087
#define RA_ELC_EVENT_SPI1_IDLE              0x088
#define RA_ELC_EVENT_SPI1_ERI               0x089
#define RA_ELC_EVENT_SPI1_TEI               0x08A
#define RA_ELC_EVENT_AES_WRREQ              0x08B
#define RA_ELC_EVENT_AES_RDREQ              0x08C
#define RA_ELC_EVENT_TRNG_RDREQ             0x08D

/* Possible peripherals to be linked to event signals */
#define RA_ELC_PERIPHERAL_GPT_A   0
#define RA_ELC_PERIPHERAL_GPT_B   1
#define RA_ELC_PERIPHERAL_GPT_C   2
#define RA_ELC_PERIPHERAL_GPT_D   3
#define RA_ELC_PERIPHERAL_ADC0    8
#define RA_ELC_PERIPHERAL_ADC0_B  9
#define RA_ELC_PERIPHERAL_DAC0    12
#define RA_ELC_PERIPHERAL_IOPORT1 14
#define RA_ELC_PERIPHERAL_IOPORT2 15
#define RA_ELC_PERIPHERAL_CTSU    18
#define RA_ELC_PERIPHERAL_DA8_0   19
#define RA_ELC_PERIPHERAL_DA8_1   20
#define RA_ELC_PERIPHERAL_SDADC0  22

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_MISC_RENESAS_RA_ELC_RA2A1_ELC_H_ */
