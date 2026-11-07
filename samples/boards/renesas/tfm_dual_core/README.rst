.. zephyr:code-sample:: renesas-tfm-dual-core
   :name: TF-M dual core

   Run TF-M on the Cortex-M85 and a non-secure Zephyr application on the Cortex-M33.

Overview
********

This sample builds three images for the Renesas RA8 dual core boards:

- TF-M, the secure image of the Cortex-M85 (CPU0).
- The Cortex-M85 non-secure image, :zephyr:code-sample:`tfm_ipc` by default.
- This application, the Cortex-M33 (CPU1) non-secure image.

TF-M serves the Cortex-M85 non-secure image only. The Cortex-M33 application is a standalone
Zephyr application, not a TF-M client, and is not managed by BL2. The Cortex-M85 non-secure image
starts the Cortex-M33 (:kconfig:option:`CONFIG_SOC_RA_ENABLE_START_SECOND_CORE`, set by the
sample's ``sysbuild.cmake``).

Requirements
************

One of the following boards:

- ``ek_ra8m2/r7ka8m2jflcac/cm33/ns``
- ``ek_ra8p1/r7ka8p1kflcac/cm33/ns``

Building and Running
********************

Build and flash all images with sysbuild:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/tfm_dual_core
   :board: ek_ra8m2/r7ka8m2jflcac/cm33/ns
   :west-args: --sysbuild
   :goals: build flash
   :compact:

``west flash`` flashes the Cortex-M85 image, merged with TF-M, first and the Cortex-M33 image
last. Use ``-DSB_CONFIG_CM85_NS_APP_PATH=<path>`` to build another Cortex-M85 non-secure
application.

Sample Output
*************

On the Cortex-M33 console:

.. code-block:: console

   Hello from CM33 non-secure: ek_ra8m2/r7ka8m2jflcac/cm33/ns
   CM33 tick 1
   CM33 tick 2

The Cortex-M85 console shows the output of the Cortex-M85 non-secure application.
