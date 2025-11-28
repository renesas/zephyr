# PSA Crypto Sample

## Overview

This sample demonstrates how to use the PSA Cryptography API in Renesas ZSDK, including key
generation, cipher, hashing, signing, verification operations.

## Requirement

For MbedTLS builds with hardware accelerator, the target board's alternative functions must be
present for this sample to work.

- [PSA Crypto Alternative Functions for Renesas](
https://gitlab.eng.renesas.com/ra-zephyr-bsp-dev/psa_crypto_driver)

## Building and Running

### Update hal_renesas, psa crypto driver, mbedtls

```shell
$ west update
```

### Build with MbedTLS Renesas Alternative (Hardware Accelerator)

```shell
$ west build -b <board> samples/psa/psa_crypto -DEXTRA_CONF_FILE=mbedtls_alt.conf
$ west flash
```

For example, build and flash on target board ek_rx74m
```shell
$ west build -b ek_rx74m samples/psa/psa_crypto -DEXTRA_CONF_FILE=mbedtls_alt.conf
$ west flash
```

### (Optional) Build with MbedTLS built in (Software library)

```shell
$ west build -b <board> samples/psa/psa_crypto
$ west flash
```

For example, build and flash on target board ek_rx74m
```shell
$ west build -b ek_rx74m samples/psa/psa_crypto
$ west flash
```

## Sample Output

```plaintext
*** Booting Zephyr OS build v4.2.0-4807-g75b9e87e3c62 ***
[00:00:00.004,000] <inf> app: PSA init success!
[00:00:00.004,000] <inf> app: AES Operation Started.
[00:00:00.004,000] <inf> app: AES Key generated Successfully.
[00:00:00.004,000] <inf> app: aead encryption and decryption completed successfully.
[00:00:00.004,000] <inf> app: Hash Operation Started.
[00:00:00.004,000] <inf> app: Hash value and Hash length comparison completed successfully.
[00:00:00.004,000] <inf> app: ECC Operation Started.
[00:00:00.024,000] <inf> app: ECC Key Pair generated Successfully.
[00:00:00.024,000] <inf> app: Exported and Destroyed Key Successfully.
[00:00:00.024,000] <inf> app: Signing the message failed as key was destroyed.
[00:00:00.024,000] <inf> app: Imported Key Successfully.
[00:00:00.064,000] <inf> app: ECC signature validated successfully.
[00:00:00.064,000] <inf> app: RSA Operation Started.
[00:00:02.965,000] <inf> app: RSA Key Pair generated Successfully.
[00:00:02.965,000] <inf> app: Exported and Destroyed Key Successfully.
[00:00:02.965,000] <inf> app: Signing the message failed as key was destroyed.
[00:00:02.966,000] <inf> app: Imported Key Successfully.
[00:00:03.069,000] <inf> app: RSA signature validated successfully.
```

## API reference

[PSA Cryptographic Operation](https://arm-software.github.io/psa-api/crypto/1.3/api/ops/index.html)

[PSA Key Management](https://arm-software.github.io/psa-api/crypto/1.3/api/keys/index.html)
