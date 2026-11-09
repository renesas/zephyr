/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef OSPI_VERIFY_H_
#define OSPI_VERIFY_H_

#include <stddef.h>
#include <stdint.h>

/* ML-DSA-87 signature size (FIPS 204) */
#define OSPI_SIG_LEN 4627

/* Verify an ML-DSA-87 signature against the public key built into this image */
int ospi_mldsa87_verify(const uint8_t *msg, size_t len, const uint8_t *sig);

/* Verify the Cortex-M85 code in OSPI flash */
int ospi_ext_code_verify(void);

/* Verify the Cortex-M33 image in OSPI flash and start the Cortex-M33 from it */
int cm33_verify_and_start(void);

#endif /* OSPI_VERIFY_H_ */
