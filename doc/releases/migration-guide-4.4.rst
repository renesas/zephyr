:orphan:

..
  See
  https://docs.zephyrproject.org/latest/releases/index.html#migration-guides
  for details of what is supposed to go into this document.

.. _migration_4.4:

Migration guide to Zephyr v4.4.0 (Working Draft)
################################################

This document describes the changes required when migrating your application from Zephyr v4.3.0 to
Zephyr v4.4.0.

Any other changes (not directly related to migrating applications) can be found in
the :ref:`release notes<zephyr_4.4>`.

.. contents::
    :local:
    :depth: 2

Build System
************

Kernel
******

Boards
******

* m5stack_fire: Removed unused pinctrl entries for UART2, and updated the UART1
  pin mapping from GPIO32/GPIO33 to GPIO16/GPIO17 to match the documented Grove
  PORT.C wiring.

* Compile definitions 'XIP_EXTERNAL_FLASH', 'USE_HYPERRAM' and 'XIP_BOOT_HEADER_XMCD_ENABLE'
  are only used in :zephyr_file:`boards/nxp/mimxrt1180_evk/xip/evkmimxrt1180_flexspi_nor_config.c`
  and :zephyr_file:`boards/nxp/mimxrt1170_evk/xmcd/xmcd.c`, we have changed them to local scope
  in the respective board CMakeLists.txt files. Applications that depended on these definitions
  being globally available may need to be updated. (:github:`101322`)

* Renesas ``ek_ra8t2/r7ka8t2lfecac/cm85`` is renamed to ``ek_ra8t2/r7ka8t2lflcac/cm85``.

* NXP has changed the scope of some in-tree compile flags to limit their visibility to only where
  they are needed. Out-of-tree applications or boards that depended on these flags being globally
  available may need to add them to their own CMakeLists.txt files to ensure they continue to build
  correctly. (:github:`100252`)
  The affected flags are listed below:

  * For the RT10xx and RT11xx families, the compile flag ``BOARD_FLASH_SIZE``, originally defined in
    ``boards/nxp/mimxrt10xx_evk/CMakeLists.txt`` and ``boards/nxp/mimxrt11xx_evk/CMakeLists.txt``, is
    used only by the HAL header ``fsl_flexspi_nor_boot.h``, which is included by
    :zephyr_file:`soc/nxp/imxrt/imxrt10xx/soc.c` and :zephyr_file:`soc/nxp/imxrt/imxrt11xx/soc.c`.
    To avoid potential collisions with other global flags, the macro is now defined at the SoC layer
    using ``zephyr_library_compile_definitions()`` in :zephyr_file:`soc/nxp/imxrt/imxrt10xx/CMakeLists.txt`
    and :zephyr_file:`soc/nxp/imxrt/imxrt11xx/CMakeLists.txt`. This change has been applied to all
    RTxxxx boards.

  * For the RTxxx family, the compile flag ``BOARD_FLASH_SIZE``, originally defined in
    ``boards/nxp/mimxrtxxx_evk/CMakeLists.txt``, is not used in the Zephyr tree and has
    therefore been removed from all RTxxx board CMakeLists.txt files.

  * For the RTxxx family, the compile flag ``BOOT_HEADER_ENABLE``, previously defined in
    ``boards/nxp/mimxrtxxx_evk/CMakeLists.txt`` and used in ``boards/nxp/rtxxx/<boot_header>.c``,
    has been replaced by a Kconfig option. Consequently, the line
    ``zephyr_compile_definitions(BOOT_HEADER_ENABLE=1)`` has been removed from the RTxxx board
    CMakeLists.txt files.

  * Removed compile flag ``BOOT_HEADER_ENABLE`` definition from :zephyr_file:`boards/nxp/rd_rw612_bga/CMakeLists.txt`,
    as it is not used in the Zephyr tree.

  * Originally, the compile flags ``XIP_BOOT_HEADER_ENABLE`` and ``XIP_BOOT_HEADER_DCD_ENABLE`` were
    used in ``boards/nxp/rt1xxx/<boot_header>.c``. These flags have been converted to Kconfig options
    across NXP RTxxxx evaluation boards, allowing boot-header configuration via the Kconfig build system
    instead of compile-time defines. Consequently, we removed ``zephyr_compile_definitions(XIP_BOOT_HEADER_ENABLE=1)``
    and ``zephyr_compile_definitions(XIP_BOOT_HEADER_DCD_ENABLE=1)`` from the RTxxxx board-level CMakeLists.txt files.
    Because these macros are also required by ``hal_nxp/rt10xx/fsl_flexspi_nor_boot.h`` and
    ``hal_nxp/rt11xx/fsl_flexspi_nor_boot.h``, they were added to the corresponding SoC-layer CMakeLists.txt files
    using ``zephyr_library_compile_definitions()`` to limit their scope.

* The following Nordic SoC Kconfigs have been deprecated and replaced, and Kconfig/CMake/code
  needs to be updated if they reference the deprecated Kconfigs:

  * :kconfig:option:`CONFIG_SOC_SERIES_NRF51X` with :kconfig:option:`CONFIG_SOC_SERIES_NRF51`
  * :kconfig:option:`CONFIG_SOC_SERIES_NRF52X` with :kconfig:option:`CONFIG_SOC_SERIES_NRF52`
  * :kconfig:option:`CONFIG_SOC_SERIES_NRF53X` with :kconfig:option:`CONFIG_SOC_SERIES_NRF53`
  * :kconfig:option:`CONFIG_SOC_SERIES_NRF54HX` with :kconfig:option:`CONFIG_SOC_SERIES_NRF54H`
  * :kconfig:option:`CONFIG_SOC_SERIES_NRF54LX` with :kconfig:option:`CONFIG_SOC_SERIES_NRF54L`
  * :kconfig:option:`CONFIG_SOC_SERIES_NRF91X` with :kconfig:option:`CONFIG_SOC_SERIES_NRF91`
  * :kconfig:option:`CONFIG_SOC_SERIES_NRF92X` with :kconfig:option:`CONFIG_SOC_SERIES_NRF92`

* ITE ``it515xx_evb`` is renamed to ``it51xxx_evb``.

Device Drivers and Devicetree
*****************************

Bluetooth
*********

Networking
**********

Other subsystems
****************

Modules
*******

Architectures
*************
