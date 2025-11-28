/*
 * Copyright (c) 2025 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

#include "psa_crypto.h"

LOG_MODULE_REGISTER(app);

#if defined(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
mbedtls_platform_context ctx = {0};
#endif

int ecc_rsa_hashing_operation(unsigned char *payload, uint8_t *payload_hash,
			      size_t *payload_hash_len)
{
	psa_status_t status = (psa_status_t)0;
	psa_hash_operation_t hash_operation = {
		0}; /* operation object to set up for Hash operation. */

	/* Calculate the hash of the message*/
	status = psa_hash_setup(&hash_operation, PSA_ALG_SHA_256);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_hash_setup API FAILED");
		return status;
	}

	/* Add a message fragment to a multipart hash operation. */
	status = psa_hash_update(&hash_operation, payload, ECC_RSA_PAYLOAD_SIZE);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_hash_update API FAILED");
		return status;
	}

	/* Finish the calculation of the hash of a message. */
	status = psa_hash_finish(&hash_operation, &payload_hash[0], PSA_HASH_MAX_SIZE,
				 payload_hash_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_hash_finish API FAILED");
		return status;
	}

	return status;
}

int aes_operation(void)
{
	psa_status_t status = PSA_SUCCESS;
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT;
	const psa_algorithm_t alg = PSA_ALG_GCM;
	psa_key_handle_t aes_key_handle = 0;
	uint8_t *output_data = NULL;
	uint8_t *output_data1 = NULL;
	size_t output_size = 0;
	size_t output_length = 0;
	size_t output_length1 = 0;

	psa_key_lifetime_t lifetime = PSA_KEY_LIFETIME_VOLATILE;
	psa_key_usage_t key_flag = (PSA_KEY_USAGE_ENCRYPT | PSA_KEY_USAGE_DECRYPT);

	static const uint8_t nonce[12] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
					  0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B};
	static const uint8_t additional_data[12] = {0};
	static const uint8_t input_data[] = {0xB9, 0x6B, 0x49, 0xE2, 0x1D, 0x62, 0x17,
					     0x41, 0x63, 0x28, 0x75, 0xDB, 0x7F, 0x6C,
					     0x92, 0x43, 0xD2, 0xD7, 0xC2};

	/* Set Key uses flags, key_algorithm, key_type, key_bits, key_lifetime, key_id */
	psa_set_key_usage_flags(&attributes, key_flag);
	psa_set_key_algorithm(&attributes, alg);
	psa_set_key_type(&attributes, PSA_KEY_TYPE_AES);
	psa_set_key_bits(&attributes, AES_KEY_BITS);
	psa_set_key_lifetime(&attributes, lifetime);

	LOG_INF("AES Operation Started.");

	/* Generating AES 256 key and allocating to key slot. */
	status = psa_generate_key(&attributes, &aes_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key API FAILED");
		return -EIO;
	}

	LOG_INF("AES Key generated Successfully.");

	/* Calculate output size for encryption and decryption. */
	output_size = sizeof(input_data) + TAG_LENGTH;
	output_data = (uint8_t *)malloc(output_size);
	output_data1 = (uint8_t *)malloc(output_size);

	if ((output_data == NULL) || (output_data1 == NULL)) {
		LOG_ERR("Out Of Memory.");
		return -ENOMEM;
	}

	/* Authenticate and encrypt */
	status = psa_aead_encrypt(aes_key_handle, alg, nonce, sizeof(nonce), additional_data,
				  sizeof(additional_data), input_data, sizeof(input_data),
				  output_data, output_size, &output_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_aead_encrypt API FAILED");
		return -EIO;
	}

	/* Authenticate and decrypt */
	status = psa_aead_decrypt(aes_key_handle, alg, nonce, sizeof(nonce), additional_data,
				  sizeof(additional_data), output_data, output_length, output_data1,
				  output_length, &output_length1);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_aead_decrypt API FAILED");
		return -EIO;
	}

	/* Compare the encrypted data length and decrypted data length. */
	if (output_length1 != sizeof(input_data)) {
		LOG_ERR("Comparison of encrypted data length and decrypted data length failed.");
		return -EINVAL;
	}

	/* Compare the encrypted data and decrypted data. */
	if (memcmp(input_data, output_data1, output_length1) != 0) {
		LOG_ERR("Comparison of encrypted data and decrypted data failed.");
		return -EINVAL;
	}

	LOG_INF("aead encryption and decryption completed successfully.");

	return 0;
}

int sha_operation(void)
{
	psa_status_t status = (psa_status_t)0;
	psa_algorithm_t alg = PSA_ALG_SHA_256; /* SHA256 algorithm. */
	psa_hash_operation_t operation = {0};  /* operation object to set up for Hash operation. */
	size_t expected_hash_len = PSA_HASH_LENGTH(alg); /* expected hash length. */
	uint8_t actual_hash[PSA_HASH_MAX_SIZE] = {0}; /* Buffer where the hash is to be written. */
	size_t actual_hash_len = 0; /* number of bytes that make up the hash value. */
	int mbed_ret_val = 0;

	/* Buffer containing the message fragment to hash. */
	const uint8_t sha256_input_data[] = {0x2e, 0x7e, 0xa8, 0x4d, 0xa4, 0xbc, 0x4d, 0x7c, 0xfb,
					     0x46, 0x3e, 0x3f, 0x2c, 0x86, 0x47, 0x05, 0x7a, 0xff,
					     0xf3, 0xfb, 0xec, 0xec, 0xa1, 0xd2, 00};

	/* Buffer containing expected hash. */
	const uint8_t sha256_expected_data[] = {0x76, 0xe3, 0xac, 0xbc, 0x71, 0x88, 0x36, 0xf2,
						0xdf, 0x8a, 0xd2, 0xd0, 0xd2, 0xd7, 0x6f, 0x0c,
						0xfa, 0x5f, 0xea, 0x09, 0x86, 0xbe, 0x91, 0x8f,
						0x10, 0xbc, 0xee, 0x73, 0x0d, 0xf4, 0x41, 0xb9};

	LOG_INF("Hash Operation Started.");
	/* Set up a multipart hash operation. */
	status = psa_hash_setup(&operation, alg);
	if (status != PSA_SUCCESS) {
		/* Hash setup failed */
		LOG_ERR("psa_hash_setup API FAILED");
		return -EIO;
	}

	/* Add a message fragment to a multipart hash operation. */
	status = psa_hash_update(&operation, sha256_input_data, sizeof(sha256_input_data));
	if (status != PSA_SUCCESS) {
		/* Hash calculation failed */
		LOG_ERR("psa_hash_update API FAILED");
		return -EIO;
	}

	/* Finish the calculation of the hash of a message. */
	status =
		psa_hash_finish(&operation, &actual_hash[0], sizeof(actual_hash), &actual_hash_len);
	if (status != PSA_SUCCESS) {
		/* Reading calculated hash failed */
		LOG_ERR("psa_hash_finish API FAILED");
		return -EIO;
	}

	/* compare hash value of calculated value with expected value. */
	mbed_ret_val = memcmp(&actual_hash[0], &sha256_expected_data[0], actual_hash_len);
	if (mbed_ret_val != 0) {
		/* Hash compare of calculated value with expected value failed */
		LOG_ERR("Hash comparison failed");
		return mbed_ret_val;
	}

	/* compare hash length of calculated value with expected value. */
	mbed_ret_val = memcmp(&expected_hash_len, &actual_hash_len, sizeof(expected_hash_len));
	if (mbed_ret_val != 0) {
		/* Hash size compare of calculated value with expected value failed */
		LOG_ERR("Hash length comparison failed");
		return mbed_ret_val;
	}

	LOG_INF("Hash value and Hash length comparison completed successfully.");

	return 0;
}

int ecc_operation(void)
{
	psa_status_t status = (psa_status_t)0;
	unsigned char payload_ecc[] =
		"ASYMMETRIC_INPUT_FOR_SIGN_ECC"; /* Buffer containing message to hash. */
	size_t signature_length = 0;             /* length of signature. */
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT; /* Contains key attributes. */
	psa_key_handle_t ecc_key_handle = {0};                     /* ECC Key handle. */
	unsigned char signature[PSA_SIGNATURE_MAX_SIZE] = {
		0}; /* Buffer containing signature for ecc. */
	uint8_t payload_hash_ecc[PSA_HASH_MAX_SIZE] = {
		0}; /* Buffer where the hash is to be written. */
	uint8_t ecc_key[ECC_256_EXPORTED_SIZE] = {
		0};                      /* Buffer where the key data is to be written. */
	size_t payload_hash_len_ecc = 0; /* number of bytes that make up the hash value. */
	size_t ecc_key_length = 0;       /* number of bytes that make up the key data. */

	LOG_INF("ECC Operation Started.");

	/* Set Key uses flags, key_algorithm, key_type, key_bits, key_lifetime, key_id */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH |
						     PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attributes, PSA_ALG_ECDSA(PSA_ALG_SHA_256));
	psa_set_key_type(&attributes,
#ifdef CONFIG_MBEDTLS_ALT
			 PSA_KEY_TYPE_ECC_KEY_PAIR_WRAPPED(PSA_ECC_FAMILY_SECP_R1)
#else
			 PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1)
#endif /* CONFIG_MBEDTLS_ALT */
	);
	psa_set_key_bits(&attributes, ECC_256_BIT_LENGTH);
	psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);

	/* Generate ECC P256R1 Key pair */
	status = psa_generate_key(&attributes, &ecc_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key API FAILED, status: %d", status);
		return -EIO;
	}
	LOG_INF("ECC Key Pair generated Successfully.");

	/* Perform Hashing operation. */
	status =
		ecc_rsa_hashing_operation(payload_ecc, &payload_hash_ecc[0], &payload_hash_len_ecc);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ecc_rsa_hashing_operation failed.");
		return -EIO;
	}

	/* Export the key. */
	status = psa_export_key(ecc_key_handle, ecc_key, sizeof(ecc_key), &ecc_key_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_export_key API FAILED");
		return -EIO;
	}

	/* Destroy the key and handle */
	status = psa_destroy_key(ecc_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_destroy_key API FAILED");
		return -EIO;
	}

	LOG_INF("Exported and Destroyed Key Successfully.");

	/* Sign message using the private key. */
	/* This is intended to fail as key was not imported after destroying. */
	status = psa_sign_hash(ecc_key_handle, PSA_ALG_ECDSA(PSA_ALG_SHA_256), payload_hash_ecc,
			       payload_hash_len_ecc, signature, sizeof(signature),
			       &signature_length);
	if (status == PSA_SUCCESS) {
		LOG_ERR("psa_sign_hash API Should Fail.");
		return -EIO;
	}
	LOG_INF("Signing the message failed as key was destroyed.");

	/* Import the previously exported key pair */
	status = psa_import_key(&attributes, ecc_key, ecc_key_length, &ecc_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_import_key API FAILED");
		return -EIO;
	}
	LOG_INF("Imported Key Successfully.");

	/* Sign message using the private key */
	status = psa_sign_hash(ecc_key_handle, PSA_ALG_ECDSA(PSA_ALG_SHA_256), payload_hash_ecc,
			       payload_hash_len_ecc, signature, sizeof(signature),
			       &signature_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_sign_hash API FAILED");
		return -EIO;
	}

	/* Verify the signature using the public key */
	status = psa_verify_hash(ecc_key_handle, PSA_ALG_ECDSA(PSA_ALG_SHA_256), payload_hash_ecc,
				 payload_hash_len_ecc, signature, signature_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_verify_hash API FAILED");
		return -EIO;
	}

	LOG_INF("ECC signature validated successfully.");

	return 0;
}

int rsa_operation(void)
{
	psa_status_t status = (psa_status_t)0;
	psa_key_handle_t rsa_key_handle = {0}; /* RSA Key handle. */
	unsigned char payload_rsa[] =
		"ASYMMETRIC_INPUT_FOR_SIGN_RSA"; /* Buffer containing message to hash. */
	unsigned char signature[PSA_SIGNATURE_MAX_SIZE] = {
		0};                  /* Buffer containing signature for rsa. */
	size_t signature_length = 0; /* length of signature. */
	psa_key_attributes_t attributes = PSA_KEY_ATTRIBUTES_INIT; /* Contains key attributes. */
	uint8_t rsa_key[RSA_2048_EXPORTED_SIZE] = {
		0};             /* Buffer where the key data is to be written. */
	size_t rsa_key_len = 0; /* number of bytes that make up the key data. */
	uint8_t payload_hash_rsa[PSA_HASH_MAX_SIZE] = {
		0};                      /* Buffer where the hash is to be written. */
	size_t payload_hash_len_rsa = 0; /* number of bytes that make up the hash value. */

	LOG_INF("RSA Operation Started.");

	/* Set Key uses flags, key_algorithm, key_type, key_bits, key_lifetime, key_id */
	psa_set_key_usage_flags(&attributes, PSA_KEY_USAGE_SIGN_HASH | PSA_KEY_USAGE_VERIFY_HASH |
						     PSA_KEY_USAGE_EXPORT);
	psa_set_key_algorithm(&attributes, PSA_ALG_RSA_PKCS1V15_SIGN_RAW);
	psa_set_key_type(&attributes,
#ifdef CONFIG_MBEDTLS_ALT
			 PSA_KEY_TYPE_RSA_KEY_PAIR_WRAPPED
#else
			 PSA_KEY_TYPE_RSA_KEY_PAIR
#endif /* CONFIG_MBEDTLS_ALT */
	);

	psa_set_key_bits(&attributes, RSA_2048_BIT_LENGTH);
	psa_set_key_lifetime(&attributes, PSA_KEY_LIFETIME_VOLATILE);

	/* Generate RSA 2048 Key pair */
	status = psa_generate_key(&attributes, &rsa_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_generate_key API FAILED");
		return -EIO;
	}

	LOG_INF("RSA Key Pair generated Successfully.");

	/* Perform Hashing operation. */
	status =
		ecc_rsa_hashing_operation(payload_rsa, &payload_hash_rsa[0], &payload_hash_len_rsa);
	if (status != PSA_SUCCESS) {
		LOG_ERR("ecc_rsa_hashing_operation failed.");
		return -EIO;
	}

	/* export the key */
	status = psa_export_key(rsa_key_handle, rsa_key, sizeof(rsa_key), &rsa_key_len);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_export_key API FAILED");
		return -EIO;
	}

	/* Destroy key. */
	status = psa_destroy_key(rsa_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_destroy_key API FAILED");
		return -EIO;
	}
	LOG_INF("Exported and Destroyed Key Successfully.");

	/* Sign message using the private key */
	/* This is intended to fail as key was not imported after destroying. */
	status = psa_sign_hash(rsa_key_handle, PSA_ALG_RSA_PKCS1V15_SIGN_RAW, payload_hash_rsa,
			       payload_hash_len_rsa, signature, sizeof(signature),
			       &signature_length);
	if (status == PSA_SUCCESS) {
		LOG_ERR("psa_sign_hash API Should Fail.");
		return -EIO;
	}
	LOG_INF("Signing the message failed as key was destroyed.");

	/* Import the key */
	status = psa_import_key(&attributes, rsa_key, rsa_key_len, &rsa_key_handle);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_import_key API FAILED");
		return -EIO;
	}

	LOG_INF("Imported Key Successfully.");
	/* Sign message using the private key */
	status = psa_sign_hash(rsa_key_handle, PSA_ALG_RSA_PKCS1V15_SIGN_RAW, payload_hash_rsa,
			       payload_hash_len_rsa, signature, sizeof(signature),
			       &signature_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_sign_hash API FAILED");
		return -EIO;
	}

	/* Verify the signature using the public key */
	status = psa_verify_hash(rsa_key_handle, PSA_ALG_RSA_PKCS1V15_SIGN_RAW, payload_hash_rsa,
				 payload_hash_len_rsa, signature, signature_length);
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_verify_hash API FAILED");
		return -EIO;
	}

	LOG_INF("RSA signature validated successfully.");

	return 0;
}

void error_handle(void)
{
	mbedtls_psa_crypto_free();
#if defined(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
	mbedtls_platform_teardown(&ctx);
	if (mbedtls_platform_setup(&ctx) == 0)
#endif
	{
		psa_crypto_init();
	}
}

int main(void)
{
	psa_status_t status;
	int ret;

#if defined(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
	if (mbedtls_platform_setup(&ctx) != 0) {
		LOG_ERR("mbedtls_platform_setup API FAILED");
		return -EIO;
	}
#endif

	status = psa_crypto_init();
	if (status != PSA_SUCCESS) {
		LOG_ERR("psa_crypto_init API FAILED: psa_status[%d]", status);
		error_handle();
	}

	LOG_INF("PSA init success!");

	ret = aes_operation();
	if (ret != 0) {
		LOG_ERR("AES operation error");
		error_handle();
	}

	ret = sha_operation();
	if (ret != 0) {
		LOG_ERR("SHA operation error");
		error_handle();
	}

	ret = ecc_operation();
	if (ret != 0) {
		LOG_ERR("ECC operation error, ret: %d", ret);
		error_handle();
	}

	ret = rsa_operation();
	if (ret != 0) {
		LOG_ERR("RSA operation error");
		error_handle();
	}

	/* De-initialize the platform. */
	mbedtls_psa_crypto_free();
#if defined(MBEDTLS_PLATFORM_SETUP_TEARDOWN_ALT)
	mbedtls_platform_teardown(&ctx);
#endif

	return 0;
}
