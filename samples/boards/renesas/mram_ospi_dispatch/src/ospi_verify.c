/*
 * Copyright (c) 2026 Renesas Electronics Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <psa/crypto.h>

#ifdef CONFIG_FLASH_RENESAS_RA_OSPI_B

#include "ospi_pubkey.h"

/* ML-DSA-87 signature size (FIPS 204) */
#define OSPI_SIG_LEN 4627

/* Signature slot, patched after link by scripts/ospi_sign.py */
const uint8_t ospi_sig_slot[OSPI_SIG_LEN] __attribute__((section(".extflash_sig"), used));

/* Provided by the linker script */
extern const uint8_t __extflash_text_reloc_start[];
extern const uint8_t __extflash_text_reloc_size[];
extern const uint8_t __extflash_sig_start[];

int ospi_image_verify(void)
{
	psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
	psa_key_id_t key = PSA_KEY_ID_NULL;
	psa_status_t st;
	int ret = -EACCES;

	st = psa_crypto_init();
	if (st != PSA_SUCCESS) {
		printk("psa_crypto_init failed (%d)\n", (int)st);
		return -EIO;
	}

	psa_set_key_type(&attr, PSA_KEY_TYPE_ML_DSA_PUBLIC_KEY);
	/* For ML-DSA the "key bits" value is the parameter set (87), not the key size */
	psa_set_key_bits(&attr, 87);
	psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_VERIFY_MESSAGE);
	psa_set_key_algorithm(&attr, PSA_ALG_DETERMINISTIC_ML_DSA);

	st = psa_import_key(&attr, ospi_pubkey, sizeof(ospi_pubkey), &key);
	if (st != PSA_SUCCESS) {
		printk("key import failed (%d)\n", (int)st);
		return -EIO;
	}

	/* ML-DSA runs in pure mode over the whole message. The OSPI window is
	 * read through memory-mapped mode, so this is the exact byte stream
	 * the CPU will fetch.
	 */
	st = psa_verify_message(key, PSA_ALG_DETERMINISTIC_ML_DSA, __extflash_text_reloc_start,
				(size_t)__extflash_text_reloc_size, __extflash_sig_start,
				OSPI_SIG_LEN);
	if (st == PSA_SUCCESS) {
		ret = 0;
	} else {
		printk("OSPI image signature invalid (%d)\n", (int)st);
	}

	psa_destroy_key(key);
	return ret;
}

#else

int ospi_image_verify(void)
{
	return 0;
}

#endif
