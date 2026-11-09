.. zephyr:code-sample:: renesas-mram-ospi-dispatch
   :name: MRAM dispatcher with signed OSPI code

   Boot MCUboot, then a dispatcher in MRAM that verifies a signed code image in
   OSPI flash before executing it in place.

Overview
********

This sample builds a three-stage boot chain for the Renesas EK-RA8M2. A
non-secure variant with TF-M is also available for the EK-RA8M2 and EK-RA8P1,
see `Non-secure build with TF-M`_.

#. **MCUboot** (MRAM) verifies the signed dispatcher image in slot0.
#. The **dispatcher** (this application, MRAM) verifies the code stored in the
   octal OSPI flash.
#. The **OSPI code** (``src/ext_code.c``) is executed in place from the OSPI
   memory-mapped window at ``0x90000000``, without being copied to RAM.

Everything except ``ext_code.c`` (``main()``, the kernel, drivers) stays in MRAM.
The OSPI code is placed with ``zephyr_code_relocate(... EXTFLASH_TEXT NOCOPY)``,
see ``CMakeLists.txt`` and ``linker_mram_ospi.ld``.

Both signatures use ML-DSA-87 (FIPS 204, pure mode) and are verified with the
PSA Crypto API (mldsa-native):

* MCUboot checks the MRAM image signed by ``imgtool``.
* The dispatcher checks the signature of the whole OSPI code section against a
  public key built into the MRAM image. The signature is stored in the
  ``.extflash_sig`` section right after the code and is patched into the image
  after link by ``scripts/ospi_sign.py``.

If the OSPI signature is invalid, ``main()`` returns without running the OSPI
code.

Requirements
************

* ``srec_cat`` (from ``srecord``) on the host.
* The ``dilithium-py`` Python package, used by ``imgtool`` and
  ``scripts/ospi_sign.py`` for ML-DSA-87 signing.

Building
********

.. code-block:: console

   west build -b ek_ra8m2/r7ka8m2jflcac/cm85 --sysbuild \
       samples/boards/renesas/mram_ospi_dispatch

By default MCUboot uses its public debug key and the OSPI image is signed with a
throw-away key generated in the build directory. Both are for development only.
To use your own ML-DSA-87 keys:

.. code-block:: console

   west build -b ek_ra8m2/r7ka8m2jflcac/cm85 --sysbuild \
       samples/boards/renesas/mram_ospi_dispatch -- \
       -DOSPI_SIGN_KEY=/path/to/ospi_mldsa87.pem \
       -DSB_CONFIG_BOOT_SIGNATURE_KEY_FILE=\"/path/to/mcuboot_mldsa87.pem\"

Non-secure build with TF-M
**************************

The dispatcher can also be built as the Cortex-M85 non-secure image for the
following boards:

* ``ek_ra8m2/r7ka8m2jflcac/cm85/ns``
* ``ek_ra8p1/r7ka8p1kflcac/cm85/ns``

In this configuration the boot chain is:

#. **TF-M BL2** (MRAM ``0x02000000``) verifies the TF-M secure image and the
   non-secure dispatcher image.
#. **TF-M** (secure) starts the **dispatcher** (non-secure, MRAM).
#. The dispatcher verifies the OSPI code and executes it in place, as in the
   secure build.

The OSPI signature check calls the PSA Crypto API in the same way, but the
calls are served by the TF-M crypto partition
(:kconfig:option:`CONFIG_PSA_CRYPTO_PROVIDER_TFM`).

TF-M takes the place of MCUboot, so build without ``--sysbuild``.
``sysbuild.conf`` adds a separate MCUboot image, which the non-secure build
does not use:

.. code-block:: console

   west build -b ek_ra8m2/r7ka8m2jflcac/cm85/ns \
       samples/boards/renesas/mram_ospi_dispatch

   west build -b ek_ra8p1/r7ka8p1kflcac/cm85/ns \
       samples/boards/renesas/mram_ospi_dispatch

BL2 signs the secure and non-secure images with ML-DSA-87. By default it uses
the TF-M development keys in ``bl2/ext/mcuboot/`` of the TF-M module. To use
your own keys, set :kconfig:option:`CONFIG_TFM_KEY_FILE_S` and
:kconfig:option:`CONFIG_TFM_KEY_FILE_NS`. Pass ``-DOSPI_SIGN_KEY`` to set the
OSPI key, as in the secure build:

.. code-block:: console

   west build -b ek_ra8m2/r7ka8m2jflcac/cm85/ns \
       samples/boards/renesas/mram_ospi_dispatch -- \
       -DOSPI_SIGN_KEY=/path/to/ospi_mldsa87.pem \
       -DCONFIG_TFM_KEY_FILE_S=\"/path/to/tfm_s_mldsa87.pem\" \
       -DCONFIG_TFM_KEY_FILE_NS=\"/path/to/tfm_ns_mldsa87.pem\"

The non-secure OSPI_B driver reprograms OCTACLK at run time. For this,
``CMakeLists.txt`` passes ``-DRENESAS_RA_CGFSAR_NS_OCTACLK=ON`` to the TF-M build
so that only OCTACLK (``CGFSAR.NONSEC17``) is made non-secure. Without it the
clock update faults.

Output images
*************

The ``split_flash`` target splits the ELF into an MRAM part and an OSPI part.
The OSPI hex and binary are byte-swapped (and padded to an even length) for the
octal read order.

.. list-table::
   :header-rows: 1

   * - File
     - Contents
     - Target
   * - ``mcuboot/zephyr/zephyr.hex``
     - MCUboot
     - MRAM ``0x02000000``
   * - ``mram_ospi_dispatch/zephyr_internal.signed.bin`` (``.hex``)
     - Signed dispatcher
     - MRAM ``0x02010000`` (slot0)
   * - ``mram_ospi_dispatch/zephyr_ospi.signed.bin`` (``.hex``)
     - OSPI code and signature
     - OSPI offset 0 (``0x90000000``)

``zephyr_merged.hex`` combines the dispatcher and OSPI hex files; it does not
contain MCUboot.

For the non-secure build there is no ``mcuboot`` image and the files are in the
build directory itself:

.. list-table::
   :header-rows: 1

   * - File
     - Contents
     - Target
   * - ``zephyr/tfm_merged.hex``
     - BL2, TF-M secure image and the BL2-signed dispatcher
     - MRAM (BL2 at ``0x02000000``, dispatcher in the ``non-secure`` slot
       at offset ``0x78000``)
   * - ``bl2.hex``
     - TF-M BL2 (copy of ``tfm/bin/bl2.hex``)
     - MRAM ``0x02000000``
   * - ``tfm_s_signed.bin`` (``.hex``)
     - BL2-signed TF-M secure image (copy of ``zephyr/tfm_s_signed.*``)
     - ``secure`` slot, MRAM offset ``0x10000``
   * - ``zephyr_internal.signed.bin`` (``.hex``)
     - BL2-signed dispatcher (copy of ``zephyr/zephyr_ns_signed.*``)
     - ``non-secure`` slot
   * - ``zephyr_ospi.signed.bin`` (``.hex``)
     - OSPI code and signature
     - OSPI offset 0 (``0x90000000``)
   * - ``zephyr_merged.hex``
     - ``zephyr/tfm_merged.hex`` and the OSPI image
     - MRAM and OSPI
   * - ``bl2.rpd``
     - TrustZone secure/non-secure boundaries (copy of ``zephyr/bl2.rpd``)
     - Device configuration

Program ``bl2.rpd`` before the MRAM images so that the TrustZone
boundaries are set. ``west flash -r rfp`` does this and then programs
``zephyr/tfm_merged.hex`` (:kconfig:option:`CONFIG_TFM_FLASH_MERGED_BINARY`),
but it does not program the OSPI image. Program ``zephyr_ospi.signed.bin`` (or
the OSPI part of ``zephyr_merged.hex``) with a programmer that supports the OSPI
flash.

Expected output
***************

On success the dispatcher prints ``External flash image verified`` and then the
addresses of the functions in MRAM, OSPI flash and SRAM. If the OSPI image is
missing or does not match its signature, it prints
``External flash image verification FAILED, not executing it`` instead. The
non-secure build prints the same messages after the TF-M boot log.
