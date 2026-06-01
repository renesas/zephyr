/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_RH850_CLOCK_COMMON_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_RH850_CLOCK_COMMON_H_

/*
 * Encoded clock ID used by the RH850 CGC clock-control driver.
 *
 * 31          28 27        16 15         8 7          0
 * +-------------+------------+------------+------------+
 * | domain      | module     | channel    | clock      |
 * +-------------+------------+------------+------------+
 *
 * These values are devicetree ABI. Keep existing numeric values stable.
 *
 * Important:
 * Do not use C unsigned suffixes such as 0U, 1U, 0xFU here.
 * This header is included by devicetree sources, and dtc expects plain
 * integer literals.
 */
#define RH850_CGC_DOMAIN_SHIFT             28
#define RH850_CGC_MODULE_SHIFT             16
#define RH850_CGC_CHANNEL_SHIFT            8
#define RH850_CGC_CLOCK_SHIFT              0

#define RH850_CGC_DOMAIN_MASK              0xf
#define RH850_CGC_MODULE_MASK              0xfff
#define RH850_CGC_CHANNEL_MASK             0xff
#define RH850_CGC_CLOCK_MASK               0xff

#define RH850_CGC_DOMAIN_PCLK              0
#define RH850_CGC_DOMAIN_SYSCLK            1
#define RH850_CGC_DOMAIN_OSC               2

#define RH850_CGC_ENCODE(domain, module, channel, clock) \
	((((domain) & RH850_CGC_DOMAIN_MASK) << RH850_CGC_DOMAIN_SHIFT) | \
	 (((module) & RH850_CGC_MODULE_MASK) << RH850_CGC_MODULE_SHIFT) | \
	 (((channel) & RH850_CGC_CHANNEL_MASK) << RH850_CGC_CHANNEL_SHIFT) | \
	 (((clock) & RH850_CGC_CLOCK_MASK) << RH850_CGC_CLOCK_SHIFT))

#define RH850_CGC_DOMAIN(id) \
	(((id) >> RH850_CGC_DOMAIN_SHIFT) & RH850_CGC_DOMAIN_MASK)

#define RH850_CGC_MODULE(id) \
	(((id) >> RH850_CGC_MODULE_SHIFT) & RH850_CGC_MODULE_MASK)

#define RH850_CGC_CHANNEL(id) \
	(((id) >> RH850_CGC_CHANNEL_SHIFT) & RH850_CGC_CHANNEL_MASK)

#define RH850_CGC_CLOCK(id) \
	(((id) >> RH850_CGC_CLOCK_SHIFT) & RH850_CGC_CLOCK_MASK)

/* Stable generic module IDs. The driver translates them to bsp_module_standby_t. */
#define RH850_MODULE_NONE                  0
#define RH850_MODULE_RSCFD                 1
#define RH850_MODULE_FLXA                  2
#define RH850_MODULE_GTM                   3
#define RH850_MODULE_ETN                   4
#define RH850_MODULE_ETNB                  5
#define RH850_MODULE_ETND                  6
#define RH850_MODULE_ETNF                  7
#define RH850_MODULE_RSENT                 8
#define RH850_MODULE_MSPI                  9
#define RH850_MODULE_RLIN3                 10
#define RH850_MODULE_ADCJ_ISO              11
#define RH850_MODULE_ADCK_ISO              12
#define RH850_MODULE_DSADC_CADC            13
#define RH850_MODULE_CXPI                  14
#define RH850_MODULE_MMCA                  15
#define RH850_MODULE_ENCA                  16
#define RH850_MODULE_PSI5                  17
#define RH850_MODULE_PSI5S                 18
#define RH850_MODULE_PWMD                  19
#define RH850_MODULE_RHSIF                 20
#define RH850_MODULE_RHSB                  21
#define RH850_MODULE_RIIC                  22
#define RH850_MODULE_SCI3                  23
#define RH850_MODULE_SFMA                  24
#define RH850_MODULE_TAPA                  25
#define RH850_MODULE_TAUD                  26
#define RH850_MODULE_TAUJ_ISO              27
#define RH850_MODULE_TPBA                  28
#define RH850_MODULE_TSG3                  29
#define RH850_MODULE_OSTM                  30
#define RH850_MODULE_ADCJ_AWO              31
#define RH850_MODULE_ADCK_AWO              32
#define RH850_MODULE_RTCA                  33
#define RH850_MODULE_TAUJ_AWO              34
#define RH850_MODULE_WDTB_AWO              35
#define RH850_MODULE_RI3C                  36
#define RH850_MODULE_SSIF                  37
#define RH850_MODULE_ATU                   38
#define RH850_MODULE_DFE                   39
#define RH850_MODULE_DFP                   40
#define RH850_MODULE_RDC                   41
#define RH850_MODULE_FCOMP                 42
#define RH850_MODULE_EMU                   43
#define RH850_MODULE_HRPWM                 44
#define RH850_MODULE_DSMIF                 45
#define RH850_MODULE_DFA                   46

/* Stable generic clock IDs. The driver translates them to fsp_priv_clock_t/cgc_clock_t. */
#define RH850_CLOCK_NONE                   0
#define RH850_CLOCK_MOSC                   1
#define RH850_CLOCK_HSIOSC                 2
#define RH850_CLOCK_LSIOSC                 3
#define RH850_CLOCK_HVIOSC                 4
#define RH850_CLOCK_SOSC                   5
#define RH850_CLOCK_PLL                    6
#define RH850_CLOCK_PLLO                   7
#define RH850_CLOCK_PLL2                   8
#define RH850_CLOCK_SSCG                   9
#define RH850_CLOCK_SYS_SSCG               10
#define RH850_CLOCK_SSCG1                  11
#define RH850_CLOCK_SYS_SSCG1              12
#define RH850_CLOCK_IOSC                   13
#define RH850_CLOCK_SYS                    14
#define RH850_CLOCK_SYS_CLEAN              15
#define RH850_CLOCK_SYS_IOSC               16
#define RH850_CLOCK_CPU                    17
#define RH850_CLOCK_SBUS                   18
#define RH850_CLOCK_HBUS                   19
#define RH850_CLOCK_UHSB                   20
#define RH850_CLOCK_HSB                    21
#define RH850_CLOCK_LSB                    22
#define RH850_CLOCK_CLKC_CPU               23
#define RH850_CLOCK_CLKC_SBUS              24
#define RH850_CLOCK_CLKC_HBUS              25
#define RH850_CLOCK_CLKC_UHSB              26
#define RH850_CLOCK_CLKC_HSB               27
#define RH850_CLOCK_CLKC_LSB               28
#define RH850_CLOCK_EMG                    29
#define RH850_CLOCK_EMCCNT                 30
#define RH850_CLOCK_ECMCNT                 31
#define RH850_CLOCK_EXTCLK0O               32
#define RH850_CLOCK_EXTCLK1O               33
#define RH850_CLOCK_MSPI                   34
#define RH850_CLOCK_RLIN                   35
#define RH850_CLOCK_RLIN3                  36
#define RH850_CLOCK_RLIN323                37
#define RH850_CLOCK_CH23_RLIN3             38
#define RH850_CLOCK_RCANOSC                39
#define RH850_CLOCK_CANFD_C                40
#define RH850_CLOCK_CANFD_XIN              41
#define RH850_CLOCK_CANXL_CH0              42
#define RH850_CLOCK_CANXL_CH1              43
#define RH850_CLOCK_ADC                    44
#define RH850_CLOCK_CLKA_ADC               45
#define RH850_CLOCK_ADCK                   RH850_CLOCK_CLKA_ADC
#define RH850_CLOCK_WDT                    46
#define RH850_CLOCK_WDTB                   47
#define RH850_CLOCK_WDTBA                  48
#define RH850_CLOCK_CLKA_WDT               49
#define RH850_CLOCK_CLKA_WDTBA             50
#define RH850_CLOCK_TAUJ                   51
#define RH850_CLOCK_CLKA_TAUJ              52
#define RH850_CLOCK_RTCA                   53
#define RH850_CLOCK_CLKA_RTCA              54
#define RH850_CLOCK_LPS                    55
#define RH850_CLOCK_CLKA_LPS               56
#define RH850_CLOCK_FLI                    57
#define RH850_CLOCK_FACI                   58
#define RH850_CLOCK_GTM                    59
#define RH850_CLOCK_SWDT                   60
#define RH850_CLOCK_CLKC_SWDT              61
#define RH850_CLOCK_DFP                    62
#define RH850_CLOCK_DFP_DIV_2              63
#define RH850_CLOCK_CLKC_SHSB              64
#define RH850_CLOCK_DFA                    65
#define RH850_CLOCK_OSPI                   66
#define RH850_CLOCK_OSPIX2                 67
#define RH850_CLOCK_PCLK                   68

#define RH850_PCLK(module, channel, clock) \
	RH850_CGC_ENCODE(RH850_CGC_DOMAIN_PCLK, RH850_MODULE_##module, channel, RH850_CLOCK_##clock)

#define RH850_SYSCLK(clock) \
	RH850_CGC_ENCODE(RH850_CGC_DOMAIN_SYSCLK, RH850_MODULE_NONE, 0, RH850_CLOCK_##clock)

#define RH850_OSC(clock) \
	RH850_CGC_ENCODE(RH850_CGC_DOMAIN_OSC, RH850_MODULE_NONE, 0, RH850_CLOCK_##clock)

/* Backward-compatible helper for old 2-argument style. Channel defaults to 0. */
#define RH850_CLOCK(module, clock) RH850_PCLK(module, 0, clock)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RENESAS_RH850_CLOCK_COMMON_H_ */
