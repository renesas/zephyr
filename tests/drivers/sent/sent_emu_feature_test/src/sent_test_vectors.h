/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENT_TEST_VECTORS_H_
#define SENT_TEST_VECTORS_H_

#include <stdint.h>

#include <zephyr/sys/util.h>

/*
 * SENT raw-period encoding used by every vector below
 * ===================================================
 *
 * The emulator array contains falling-edge-to-falling-edge periods in SENT
 * ticks. It does not contain high time or low time separately.
 *
 * Calibration pulse:
 *
 *	 period = 56 ticks
 *
 * Status, data and CRC nibbles:
 *
 *	 period_ticks = 12 + nibble_value
 *
 * Examples:
 *
 *	 nibble 0x0 -> 12 + 0  = 12 ticks
 *	 nibble 0x1 -> 12 + 1  = 13 ticks
 *	 nibble 0x5 -> 12 + 5  = 17 ticks
 *	 nibble 0xF -> 12 + 15 = 27 ticks
 *
 * Each ordinary frame in this test has:
 *
 *	 [calibration]
 *	 [status]
 *	 [data0] [data1] [data2] [data3] [data4] [data5]
 *	 [CRC]
 *
 * Therefore one frame has 9 raw period entries.
 */

/*
 * Known-good FSP reference vector
 * ===============================
 */
static const uint32_t fsp_enhanced_serial_format_0_pulses[] = {
	/*
	 * Dummy frame
	 */
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U,

	56U, 23U, 12U, 12U, 12U, 12U, 12U, 12U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 13U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 14U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 15U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 16U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 17U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 18U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 19U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 20U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 21U, 17U,
	56U, 27U, 12U, 12U, 12U, 12U, 12U, 22U, 17U,
	56U, 27U, 12U, 12U, 12U, 12U, 12U, 23U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 24U, 17U,
	56U, 23U, 12U, 12U, 12U, 12U, 12U, 25U, 17U,
	56U, 27U, 12U, 12U, 12U, 12U, 12U, 26U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 12U, 27U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 13U, 12U, 17U,
	56U, 19U, 12U, 12U, 12U, 12U, 13U, 13U, 17U,

	/* Stream termination / following calibration periods. */
	56U,
	56U,
};

static const uint8_t expected_final_nibble[19] = {
	/* dummy */
	0x0,

	/* ES 1..18 */
	0x0, 0x1, 0x2, 0x3,
	0x4, 0x5, 0x6, 0x7,
	0x8, 0x9, 0xA, 0xB,
	0xC, 0xD, 0xE, 0xF,
	0x0, 0x1,
};

static const uint8_t expected_nibble4[19] = {
	/* dummy */
	0x0,

	/* ES 1..16 */
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,
	0x0, 0x0, 0x0, 0x0,

	/* ES 17..18 */
	0x1, 0x1,
};

#define FSP_REFERENCE_FRAME_COUNT 19U
#define FSP_REFERENCE_PERIODS_PER_FRAME 9U
#define FSP_REFERENCE_TERMINATION_COUNT 2U

BUILD_ASSERT(
	ARRAY_SIZE(fsp_enhanced_serial_format_0_pulses) ==
		((FSP_REFERENCE_FRAME_COUNT * FSP_REFERENCE_PERIODS_PER_FRAME) +
		 FSP_REFERENCE_TERMINATION_COUNT),
	"Unexpected FSP reference-vector length");

/*
 * Known-good single frame
 * =======================
 *
 * This is the first/dummy frame from the known-good FSP vector, followed by
 * two calibration periods:
 *
 *	 status = 0x0
 *	 data   = {0, 0, 0, 0, 0, 0}
 *	 CRC raw field = 0x5
 *
 * Raw conversion:
 *
 *	 0x0 -> 12 ticks
 *	 0x5 -> 17 ticks
 */
static const uint32_t known_good_single_frame[] = {
	56U,
	12U,
	12U, 12U, 12U, 12U, 12U, 12U,
	17U,
	56U,
	56U,
};

/*
 * Calibration-error frame
 * =======================
 *
 * Identical to known_good_single_frame except calibration is deliberately
 * changed from 56 ticks to 30 ticks. The receiver should reject this frame.
 */
static const uint32_t bad_calibration_single_frame[] = {
	30U,
	12U,
	12U, 12U, 12U, 12U, 12U, 12U,
	17U,
	56U,
	56U,
};

/*
 * Explicit CRC vectors
 * ====================
 *
 * Payload used for CRC validation:
 *
 *	 status = 0x0
 *	 data   = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6}
 *
 * Data periods:
 *
 *	 0x1 -> 13
 *	 0x2 -> 14
 *	 0x3 -> 15
 *	 0x4 -> 16
 *	 0x5 -> 17
 *	 0x6 -> 18
 *
 * CRC calculation recorded here for later validation:
 *
 *	 polynomial = x^4 + x^3 + x^2 + 1 = 0x0D
 *	 seed	   = 0x05
 *
 */
static const uint32_t recommended_crc_single_frame[] = {
	56U,
	12U,
	13U, 14U, 15U, 16U, 17U, 18U,
	14U,
	56U,
	56U,
};

static const uint32_t legacy_crc_single_frame[] = {
	56U,
	12U,
	13U, 14U, 15U, 16U, 17U, 18U,
	25U,
	56U,
	56U,
};

/*
 * Corrupted CRC candidates
 * ========================
 *
 * Only the CRC period is changed. The corruption uses CRC nibble XOR 0x1:
 *
 *	 recommended: 0x1 XOR 0x1 = 0x0 -> 12 ticks
 *	 legacy:	  0x8 XOR 0x1 = 0x9 -> 21 ticks
 */
static const uint32_t bad_recommended_crc_single_frame[] = {
	56U,
	12U,
	13U, 14U, 15U, 16U, 17U, 18U,
	12U,
	56U,
	56U,
};

static const uint32_t bad_legacy_crc_single_frame[] = {
	56U,
	12U,
	13U, 14U, 15U, 16U, 17U, 18U,
	21U,
	56U,
	56U,
};

/*
 * Variable pause-pulse vector
 * ===========================
 *
 * Four known-good zero-data frames are followed by explicit pause periods.
 * The pause is inserted after the CRC period and before the next calibration
 * period. The four pause periods are intentionally different:
 *
 *	 frame 0 pause = 32 ticks
 *	 frame 1 pause = 40 ticks
 *	 frame 2 pause = 48 ticks
 *	 frame 3 pause = 64 ticks
 *
 * Frame layout with pause:
 *
 *	 56, status, data0..data5, CRC, pause
 */
static const uint32_t variable_pause_frames[] = {
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 32U,
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 40U,
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 48U,
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 64U,
	56U,
	56U,
};

/*
 * Fixed pause-pulse vector
 * ========================
 *
 * Same safe zero-data frame repeated four times. Every frame uses the same
 * explicit 64-tick pause period.
 */
static const uint32_t fixed_pause_frames[] = {
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 64U,
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 64U,
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 64U,
	56U, 12U, 12U, 12U, 12U, 12U, 12U, 12U, 17U, 64U,
	56U,
	56U,
};

#endif /* SENT_TEST_VECTORS_H_ */
