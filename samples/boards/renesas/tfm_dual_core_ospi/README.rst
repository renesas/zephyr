.. zephyr:code-sample:: renesas-tfm-dual-core-ospi
   :name: TF-M dual core with a verified Cortex-M33 image in OSPI flash

   Run TF-M on the Cortex-M85 and start a Cortex-M33 image executed in place from OSPI flash only
   after verifying its ML-DSA-87 signature.

Overview
********

This sample builds four images for the Renesas EK-RA8P1 with sysbuild:

- BL2 and TF-M, the secure images of the Cortex-M85 (CPU0), in MRAM.
- The Cortex-M85 non-secure image (``cm85/``), in MRAM. It owns the OSPI_B controller and
  verifies the code in OSPI flash before anything runs from it.
- This application, the Cortex-M33 (CPU1) non-secure image, linked into OSPI flash and executed
  in place.

Boot sequence:

#. BL2 verifies and boots TF-M and the Cortex-M85 non-secure image.
#. The OSPI_B driver of the Cortex-M85 puts the flash in memory-mapped mode.
#. The Cortex-M85 verifies its own OSPI code (``cm85/src/ext_code.c``) and runs it.
#. The Cortex-M85 reads the Cortex-M33 manifest, verifies the signature of the Cortex-M33 image,
   writes the image address to ``CPU1INITVTOR`` and releases the Cortex-M33. If the
   verification fails the Cortex-M33 is never started.

:kconfig:option:`CONFIG_SOC_RA_ENABLE_START_SECOND_CORE` is not used: it starts the Cortex-M33
from MRAM in ``soc_late_init_hook()``, before ``main()`` could verify anything.

Both OSPI images are signed with the same ML-DSA-87 key (FIPS 204, pure mode). Its public key is
built into the Cortex-M85 MRAM image and the verification runs in the TF-M crypto partition
through the PSA Crypto API.

OSPI flash layout
*****************

The layout is defined once in ``ospi_layout.dtsi`` and included by both images:

.. list-table::
   :header-rows: 1

   * - Partition
     - Offset
     - Size
     - Contents
   * - ``cm85_ospi_partition``
     - ``0x0000000``
     - 1 MB
     - Cortex-M85 OSPI code and its signature
   * - ``cm33_ospi_partition``
     - ``0x0100000``
     - 1016 KB
     - Cortex-M33 image, vector table first
   * - ``cm33_ospi_manifest_partition``
     - ``0x01fe000``
     - 8 KB
     - Cortex-M33 manifest
   * - ``auto_calib_partition``
     - ``0x3fff000``
     - 4 KB
     - OSPI_B auto-calibration pattern

The Cortex-M33 manifest is written by ``scripts/ospi_sign.py manifest`` (little-endian):

.. list-table::
   :header-rows: 1

   * - Offset
     - Size
     - Field
   * - 0
     - 4
     - Magic ``0x33334D43`` ("CM33")
   * - 4
     - 4
     - Image size in bytes
   * - 8
     - 4627
     - ML-DSA-87 signature of the image

Requirements
************

* ``ek_ra8p1/r7ka8p1kflcac/cm33/ns``
* ``srec_cat`` (from ``srecord``) on the host.
* The ``dilithium-py`` Python package, used by ``imgtool`` and ``scripts/ospi_sign.py``.

Building
********

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/tfm_dual_core_ospi
   :board: ek_ra8p1/r7ka8p1kflcac/cm33/ns
   :west-args: --sysbuild
   :goals: build
   :compact:

By default the OSPI images are signed with a throw-away key generated in the build directory.
To use your own ML-DSA-87 key:

.. code-block:: console

   west build -b ek_ra8p1/r7ka8p1kflcac/cm33/ns --sysbuild \
       samples/boards/renesas/tfm_dual_core_ospi -- -DOSPI_SIGN_KEY=/path/to/ospi_mldsa87.pem

Output images
*************

The OSPI hex files are byte-swapped (and padded to an even length) for the octal read order.

.. list-table::
   :header-rows: 1

   * - File
     - Contents
   * - ``merged.hex``
     - Everything below, for flashing the whole device
   * - ``cm85_ns/zephyr_merged.hex``
     - BL2, TF-M, Cortex-M85 non-secure image (MRAM) and Cortex-M85 OSPI code
   * - ``tfm_dual_core_ospi/cm33_ospi.hex``
     - Cortex-M33 image and manifest (OSPI)

Flash ``merged.hex`` with a programmer that supports the EK-RA8P1 OSPI flash. ``west flash``
programs ``zephyr.hex`` of each image and does not program the byte-swapped OSPI images.

Sample Output
*************

On the Cortex-M85 console:

.. code-block:: console

   External flash image verified
   CM33 image verified, CM33 started

On the Cortex-M33 console:

.. code-block:: console

   Hello from CM33 non-secure, running from OSPI: ek_ra8p1/r7ka8p1kflcac/cm33/ns
   CM33 tick 1
