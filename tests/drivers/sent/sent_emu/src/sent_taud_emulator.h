/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SENT_TAUD_EMULATOR_H_
#define SENT_TAUD_EMULATOR_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the TAUD-based SENT waveform emulator.
 *
 * This function configures the TAUD master and slave channels, applies
 * output pinctrl, connects the TAUD interrupts, and enables the output.
 *
 * @retval 0       Emulator initialized successfully.
 * @retval -EALREADY Emulator is already initialized.
 * @retval other   Error returned by pinctrl configuration.
 */
int sent_taud_emulator_open(void);

/**
 * @brief Transmit a raw array of SENT pulse lengths.
 *
 * Every array element represents one falling-edge-to-falling-edge period
 * expressed in SENT ticks.
 *
 * For example, with CONFIG_SENT_TAUD_TICK_US=3:
 *
 *     pulse_ticks[i] = 56
 *
 * produces:
 *
 *     56 * 3 us = 168 us
 *
 * Unlike the original FSP emulator, this API does not require an explicit
 * end marker. The number of pulse periods is supplied by @p count.
 *
 * @param[in] pulse_ticks Raw SENT pulse-length array.
 * @param[in] count       Number of entries in @p pulse_ticks.
 *
 * @retval 0          Transmission completed successfully.
 * @retval -EACCES    Emulator has not been opened.
 * @retval -EINVAL    Invalid pointer, zero count, or zero pulse length.
 * @retval -ERANGE    A pulse exceeds the TAUD counter range.
 * @retval -ETIMEDOUT Transmission did not finish before the configured
 *                    timeout.
 */
int sent_taud_emulator_send_raw(
	const uint32_t *pulse_ticks,
	size_t count);

/**
 * @brief Stop and close the TAUD-based SENT waveform emulator.
 *
 * @retval 0 Emulator closed successfully.
 */
int sent_taud_emulator_close(void);

#ifdef __cplusplus
}
#endif

#endif /* SENT_TAUD_EMULATOR_H_ */
