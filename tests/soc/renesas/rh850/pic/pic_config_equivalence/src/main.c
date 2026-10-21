/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Register-level test of the RH850 PIC driver.
 *
 * The overlay of the board (boards/<board>.overlay) writes a few ADC trigger routes in the pic0
 * node. After boot the driver has called R_PIC_Open() once, so the ADC trigger registers of PIC20
 * and PIC21 must hold exactly the values of the tables below, and EVERY register that is not
 * listed there must be 0 (the reset value).
 *
 * Register names (iodefine of the FSP):
 *   ADCJ (U2A)
 *     R_PIC20->PIC2ADCJ[adc].TSEL[sg] / .EDGSEL, R_PIC20->PIC2ADTEN4[taud].SG[sg].SG
 *   ADCK (U2B, U2C)
 *     R_PIC20->PIC20ADCK[adc].TSEL[sg] / .EDGSEL, R_PIC20->PIC20ADTEN4[taud].SG[sg].SG
 *     R_PIC21->PIC21ADCK[adc].TSEL[sg] / .EDGSEL, R_PIC21->PIC21ADTEN4[taud].SG[sg].SG
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/ztest.h>

#include <r_pic_api.h>
#include <r_pic.h>

#define PIC_NODE DT_NODELABEL(pic0)

/* Size of the tables: the largest MCU (U2B has 4 ADC units, 6 scan groups, 2 + 2 TAUD units) */
#define MAX_ADC  4
#define MAX_SG   6
#define MAX_TAUD 2

/* Expected register values of this board (everything not listed is 0) */

/*
 * sel20 / sel21: [adc][sg]   level 1, source bits
 * edg20 / edg21: [adc]       2 bits per scan group: 0 rising, 1 falling, 2 both
 * ch20 / ch21:   [taud][sg]  level 2, TAUD channel mask (ch20: TAUD0, TAUD1; ch21: TAUD2, TAUD3)
 */
#if BSP_FEATURE_PIC_ADCJ_TRIG_SEL
/* U2A (PIC20 only): ADC0 SG0 RISING TAUD0_CH2; ADC0 SG1 BOTH TAUD0_CH2 and TAUD1_CH5 */
static const uint32_t sel20[MAX_ADC][MAX_SG] = {[0] = {[0] = 0x1U, [1] = 0x3U}};
static const uint16_t edg20[MAX_ADC] = {[0] = 0x8U}; /* SG1 both (2) << 2 */
static const uint16_t ch20[MAX_TAUD][MAX_SG] = {
	[0] = {[0] = 0x4U, [1] = 0x4U}, [1] = {[1] = 0x20U}
};
#elif (BSP_FEATURE_PIC21_TAUD_NUMBER_UNIT == 2)
/* U2B: ADC0 SG0 RISING TAUD0_CH2; ADC1 SG1 BOTH TAUD0_CH2, TAUD2_CH2 and TAUD2_CH3 */
static const uint32_t sel20[MAX_ADC][MAX_SG] = {[0] = {[0] = 0x1U}, [1] = {[1] = 0x1U}};
static const uint16_t edg20[MAX_ADC] = {[1] = 0x8U};
static const uint16_t ch20[MAX_TAUD][MAX_SG] = {[0] = {[0] = 0x4U, [1] = 0x4U}};
static const uint32_t sel21[MAX_ADC][MAX_SG] = {[1] = {[1] = 0x1U}};
static const uint16_t edg21[MAX_ADC] = {[1] = 0x8U};
static const uint16_t ch21[MAX_TAUD][MAX_SG] = {[0] = {[1] = 0xCU}};
#else
/* U2C: ADC0 SG0 RISING TAUD0_CH2; ADC2 SG1 BOTH TAUD2_CH2 */
static const uint32_t sel20[MAX_ADC][MAX_SG] = {[0] = {[0] = 0x1U}};
static const uint16_t edg20[MAX_ADC];
static const uint16_t ch20[MAX_TAUD][MAX_SG] = {[0] = {[0] = 0x4U}};
static const uint32_t sel21[MAX_ADC][MAX_SG] = {[2] = {[1] = 0x1U}};
static const uint16_t edg21[MAX_ADC] = {[2] = 0x8U};
static const uint16_t ch21[MAX_TAUD][MAX_SG] = {[0] = {[1] = 0x4U}};
#endif

/* Register access of this MCU family */
#if BSP_FEATURE_PIC_ADCJ_TRIG_SEL
#define REG_SEL20(a, s) (R_PIC20->PIC2ADCJ[a].TSEL[s])
#define REG_EDG20(a)    (R_PIC20->PIC2ADCJ[a].EDGSEL)
#define REG_CH20(k, s)  (R_PIC20->PIC2ADTEN4[k].SG[s].SG)
#else
#define REG_SEL20(a, s) (R_PIC20->PIC20ADCK[a].TSEL[s])
#define REG_EDG20(a)    (R_PIC20->PIC20ADCK[a].EDGSEL)
#define REG_CH20(k, s)  (R_PIC20->PIC20ADTEN4[k].SG[s].SG)
#define REG_SEL21(a, s) (R_PIC21->PIC21ADCK[a].TSEL[s])
#define REG_EDG21(a)    (R_PIC21->PIC21ADCK[a].EDGSEL)
#define REG_CH21(k, s)  (R_PIC21->PIC21ADTEN4[k].SG[s].SG)
#endif

/* R_PIC_Open() succeeded: the init function returned 0, so the device is ready */
ZTEST(pic_config_equivalence, test_pic_device_is_ready)
{
	zassert_true(device_is_ready(DEVICE_DT_GET(PIC_NODE)),
		     "R_PIC_Open() failed, the PIC device is not ready");
}

/* Level 1 (source bits) of PIC20 and PIC21 */
ZTEST(pic_config_equivalence, test_adc_level1_sources)
{
	for (int a = 0; a < BSP_FEATURE_PIC_ADC_NUMBER_UNIT; a++) {
		for (int s = 0; s < BSP_FEATURE_PIC_ADC_NUMBER_SG; s++) {
			zassert_equal(REG_SEL20(a, s), sel20[a][s],
				      "PIC20 TSEL adc %d sg %d", a, s);
#if !BSP_FEATURE_PIC_ADCJ_TRIG_SEL
			zassert_equal(REG_SEL21(a, s), sel21[a][s],
				      "PIC21 TSEL adc %d sg %d", a, s);
#endif
		}
	}
}

/* Level 2 (TAUD channel masks): no ADC index, shared by every ADC unit */
ZTEST(pic_config_equivalence, test_adc_level2_taud_channels)
{
	for (int k = 0; k < BSP_FEATURE_PIC20_TAUD_NUMBER_UNIT; k++) {
		for (int s = 0; s < BSP_FEATURE_PIC_ADC_NUMBER_SG; s++) {
			zassert_equal(REG_CH20(k, s), ch20[k][s],
				      "PIC20 ADTEN4 taud %d sg %d", k, s);
		}
	}
#if !BSP_FEATURE_PIC_ADCJ_TRIG_SEL
	for (int k = 0; k < BSP_FEATURE_PIC21_TAUD_NUMBER_UNIT; k++) {
		for (int s = 0; s < BSP_FEATURE_PIC_ADC_NUMBER_SG; s++) {
			zassert_equal(REG_CH21(k, s), ch21[k][s],
				      "PIC21 ADTEN4 taud %d sg %d", k, s);
		}
	}
#endif
}

/* Edge: one 2-bit field per scan group, written only to a register with a source of the group */
ZTEST(pic_config_equivalence, test_adc_edges)
{
	for (int a = 0; a < BSP_FEATURE_PIC_ADC_NUMBER_UNIT; a++) {
		zassert_equal(REG_EDG20(a), edg20[a], "PIC20 EDGSEL adc %d", a);
#if !BSP_FEATURE_PIC_ADCJ_TRIG_SEL
		zassert_equal(REG_EDG21(a), edg21[a], "PIC21 EDGSEL adc %d", a);
#endif
	}
}

ZTEST_SUITE(pic_config_equivalence, NULL, NULL, NULL, NULL, NULL);
