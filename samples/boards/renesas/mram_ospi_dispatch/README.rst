.. zephyr:code-sample:: renesas-mram-ospi-dispatch
   :name: MRAM dispatcher with signed OSPI code

   Boot MCUboot, then a dispatcher in MRAM that verifies a signed code image in
   OSPI flash before executing it in place.

Overview
********

This sample builds a three-stage boot chain for the Renesas EK-RA8M2:

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

Expected output
***************

On success the dispatcher prints ``External flash image verified`` and then the
addresses of the functions in MRAM, OSPI flash and SRAM. If the OSPI image is
missing or does not match its signature, it prints
``External flash image verification FAILED, not executing it`` instead.
