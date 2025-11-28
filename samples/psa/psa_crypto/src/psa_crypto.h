/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef PSA_CRYPTO_H_
#define PSA_CRYPTO_H_

#include <mbedtls/platform.h>
#include <psa/crypto.h>
#include <psa/crypto_values.h>
#ifdef CONFIG_MBEDTLS_ALT
#include <vendor.h>
#endif

/* Macro definitions.*/
#define ECC_256_BIT_LENGTH     (256U)
#define RSA_2048_BIT_LENGTH    (2048U)
#define RSA_2048_EXPORTED_SIZE (1210U)
#define ECC_256_EXPORTED_SIZE  (500U)
#define AES_256_EXPORTED_SIZE  (500U)
#define TAG_LENGTH             ((size_t)16U)
#define AES_KEY_BITS           (256U)
#define ECC_RSA_PAYLOAD_SIZE   (30U)

#endif /* PSA_CRYPTO_H_ */
