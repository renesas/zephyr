.. zephyr:board:: ek_ra8p1

Overview
********

The EK-RA8P1 is an Evaluation Kit for Renesas RA8P1 MCU Group which integrates multiple series of software-compatible
Arm®-based 32-bit cores that share a common set of Renesas peripherals to facilitate design scalability and efficient
platform-based product development.

The MCU in this series incorporates a high-performance Arm® Cortex®-M85 core running up to 1 GHz and Arm®
Cortex®-M33 core running up to 250 MHz with the following features:

- Up to 1 MB MRAM
- 2 MB SRAM (256 KB of CM85 TCM RAM, 128 KB CM33 TCM RAM, 1664 KB of user SRAM)
- Arm® Ethos™-U55 NPU
- Octal Serial Peripheral Interface (OSPI)
- Layer 3 Ethernet Switch Module (ESWM), USBFS, USBHS, SD/MMC Host Interface
- Graphics LCD Controller (GLCDC)
- 2D Drawing Engine (DRW)
- MIPI DSI/CSI interface
- Analog peripherals
- Security and safety features

**MCU Native Pin Access**

- 1 GHz Arm Cortex-M85 and 250 MHz Arm Cortex-M33 based RA8P1 MCU in 289 pins, BGA package
- Native pin access through 2 x 20-pin, and 2 x 40-pin headers (no populated)
- Camera Expansion connector (present on the underside of the EK-RA8P1 board)
- 2-Lane MIPI Display connector (present on the underside of the EK-RA8P1 board)
- Parallel graphics display interface connector
- MCU current measurement points for precision current consumption measurement
- Multiple clock sources - RA8P1 MCU oscillator and sub-clock oscillator crystals,
  providing precision 24.000 MHz and 32,768 Hz reference clocks.
  Additional low precision clocks are available internal to the RA8P1 MCU

**System Control and Ecosystem Access**

- USB Full Speed Host and Device (USB-C connector)
- Four 5V input sources

  - USB (Debug, Full Speed, High Speed)
  - External power supply (using surface mount clamp test points and power input vias)

- Three Debug modes

  - Debug on-board (SWD and JTAG)
  - Debug in (ETM, SWD, SWO and JTAG)
  - Debug out (SWD, SWO, and JTAG)

- User LEDs and buttons

  - Three User LEDs (red, blue, green)
  - Power LED (white) indicating availability of regulated power
  - Debug LED (yellow) indicating the debug connection
  - Ethernet LEDs (amber, yellow, green)
  - Two User buttons
  - One Reset button

- Five most popular ecosystems expansions

  - Two Seeed Grove system (I2C/I3C/Analog) connectors (not populated)
  - One SparkFun Qwiic connector (not populated)
  - Two Digilent Pmod (SPI, UART and I2C) connectors
  - Arduino (Uno R3) connector
  - MikroElektronika mikroBUS connector (not populated)

- MCU boot configuration jumper

**Special Feature Access**

- Ethernet (RJ45 RGMII interface)
- USB High Speed Host and Device (USB-C connector)
- 512 Mb (64 MB) External Octo-SPI Flash (present in the MCU Native Pin Access area of the EK-RA8P1 board)

Hardware
********

Detailed hardware features can be found at:

- RA8P1 MCU: `RA8P1 Group User's Manual Hardware`_
- EK-RA8P1 board: `EK-RA8P1 - User's Manual`_

Supported Features
==================

.. zephyr:board-supported-hw::

.. note::

   - For using the Camera Expansion Port (J35) in DVP interface, please set switch SW4 as following configuration:

     +-------------+-------------+----------------+---------------+-----------+------------+-------------+-------------+
     | SW4-1 PMOD1 | SW4-2 PMOD1 | SW4-3 Octo-SPI | SW4-4 Arduino | SW4-5 I3C | SW4-6 MIPI | SW4-7 USBFS | SW4-8 USBHS |
     +-------------+-------------+----------------+---------------+-----------+------------+-------------+-------------+
     |     OFF     |     OFF     |      OFF       |     OFF       |     OFF   |     ON     |     OFF     |    OFF      |
     +-------------+-------------+----------------+---------------+-----------+------------+-------------+-------------+

Dual Core Operation
*******************

The EK-RA8P1 supports dual core operation with both the Cortex-M85 (CPU0) and Cortex-M33 (CPU1) cores.
By default, the CM85 core is the boot core and is responsible for initializing the system and
starting the CM33 core.

Memory Usage
============

By default, MRAM (Flash) and SRAM are split evenly between the two cores.
Users can manually change the address and size for MRAM (Flash) and SRAM as follows node:

   - CPU0: &code_mram_cm85, &sram0
   - CPU1: &code_mram_cm33, &sram1

.. note::

   - MRAM usable range: 0x0200_0000 ... 0x0210_0000 (1 MB)
   - SRAM usable range: 0x2200_0000 ... 0x221A_0000 (1664 KB)

Dual Core Flashing
==================

When flashing or debugging dual-core samples, ensure that CONFIG_SOC_RA_ENABLE_START_SECOND_CORE is selected
for the CM85 image. The CM85 core is responsible for starting the CM33 core in soc_late_init_hook.

Dual Core Operation with TF-M
=============================

With ``ek_ra8p1/r7ka8p1kflcac/cm85/ns``, TF-M runs on the CM85 core only and serves the CM85
non-secure image. The CM33 core can run a standalone Zephyr application built for
``ek_ra8p1/r7ka8p1kflcac/cm33/ns``. This application is not a TF-M client and is not managed by
BL2. The CM85 non-secure image starts the CM33 core
(:kconfig:option:`CONFIG_SOC_RA_ENABLE_START_SECOND_CORE`).

The :zephyr:code-sample:`renesas-tfm-dual-core` sample builds TF-M, the CM85 non-secure image
(:zephyr:code-sample:`tfm_ipc` by default) and the CM33 non-secure image with sysbuild:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/tfm_dual_core
   :board: ek_ra8p1/r7ka8p1kflcac/cm33/ns
   :west-args: --sysbuild
   :goals: build flash

``west flash`` flashes the CM85 image, merged with TF-M, first and the CM33 image last. Use
``-DSB_CONFIG_CM85_NS_APP_PATH=<path>`` to build another CM85 non-secure application.

.. note::

   The TrustZone partition settings are programmed together with the CM85 non-secure image. They
   must be loaded before the CM33 non-secure image is flashed, so flash the CM85 non-secure image
   first. With sysbuild, the application is responsible for setting this order in its
   ``sysbuild.cmake``, for example with
   ``sysbuild_add_dependencies(FLASH <cm33_image> <cm85_image>)``, as the
   :zephyr:code-sample:`renesas-tfm-dual-core` sample does.

.. note::

   The security extension of the CM33 core (CPU1) must be disabled with the Renesas Flash
   Programmer (RFP) tool before flashing the ``cm33/ns`` image.

The two images can also be built and flashed separately, flashing the CM85 image first:

.. code-block:: console

   west build -b ek_ra8p1/r7ka8p1kflcac/cm33/ns <cm33_app> -d build_cm33
   west build -b ek_ra8p1/r7ka8p1kflcac/cm85/ns <cm85_app> -d build_cm85
   west flash -d build_cm85
   west flash -d build_cm33

Programming and Debugging
*************************

.. zephyr:board-supported-runners::

Applications for the ``ek_ra8p1`` board configuration can be
built, flashed, and debugged in the usual way. See
:ref:`build_an_application` and :ref:`application_run` for more details on
building and running.

Here is an example for the :zephyr:code-sample:`hello_world` application on CM85 core.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: ek_ra8p1/r7ka8p1kflcac/cm85
   :goals: flash

Open a serial terminal, reset the board (press the S3 button), and you should
see the following message in the terminal:

.. code-block:: console

   ***** Booting Zephyr OS v4.2.0-xxx-xxxxxxxxxxxxx *****
   Hello World! ek_ra8p1/r7ka8p1kflcac/cm85

For the CM33 core, you can use the ``--sysbuild`` flow to build a minimal first-core launcher image that
starts the CM33 core.

.. zephyr-app-commands::
   :tool: west
   :zephyr-app: samples/hello_world
   :board: ek_ra8p1/r7ka8p1kflcac/cm33
   :goals: build flash
   :west-args: --sysbuild

Flashing
========

Program can be flashed to EK-RA8P1 via the on-board SEGGER J-Link debugger.
SEGGER J-link's drivers are available at https://www.segger.com/downloads/jlink/

To flash the program to board

1. Connect to J-Link OB via USB port to host PC

2. Make sure J-Link OB jumper is in default configuration as described in `EK-RA8P1 - User's Manual`_

3. Execute west command

	.. code-block:: console

		west flash -r jlink

4. Or flash using the Renesas Flash Programmer (rfp) runner. Check that ``rfp-cli`` is available,
   or set its path with ``--rfp-cli``.

   .. code-block:: console

      west flash -r rfp

   For ``cm85/ns`` targets built with TF-M, the rfp runner first programs the partition data file
   generated by TF-M (``build/zephyr/bl2.rpd``, or ``build/zephyr/tfm_s.rpd`` without BL2) to set
   the TrustZone secure / non-secure boundaries, then erases the whole chip (``-erase-chip``) and
   programs the merged image.

For ``cm85/ns`` targets built with TF-M, :kconfig:option:`CONFIG_TFM_FLASH_MERGED_BINARY` is
enabled by default, so all runners program ``build/zephyr/tfm_merged.hex``, which contains BL2
(when :kconfig:option:`CONFIG_TFM_BL2` is enabled), the TF-M secure image and the Zephyr
non-secure image, instead of the non-secure image only.

MCUboot bootloader
==================

The sysbuild makes possible to build and flash all necessary images needed to
bootstrap the board.

To build the sample application using sysbuild use the command:

.. zephyr-app-commands::
   :tool: west
   :zephyr-app: samples/hello_world
   :board: ek_ra8p1/r7ka8p1kflcac/cm85
   :goals: build flash
   :west-args: --sysbuild
   :gen-args: -DSB_CONFIG_BOOTLOADER_MCUBOOT=y

By default, Sysbuild creates MCUboot and user application images.

Build directory structure created by sysbuild is different from traditional
Zephyr build. Output is structured by the domain subdirectories:

.. code-block::

  build/
  ├── hello_world
  |    └── zephyr
  │       ├── zephyr.elf
  │       ├── zephyr.hex
  │       ├── zephyr.bin
  │       ├── zephyr.signed.bin
  │       └── zephyr.signed.hex
  ├── mcuboot
  │    └── zephyr
  │       ├── zephyr.elf
  │       ├── zephyr.hex
  │       └── zephyr.bin
  └── domains.yaml

.. note::

   With ``--sysbuild`` option, MCUboot will be rebuilt and re-flashed
   every time the pristine build is used.

To only flash the user application in the subsequent builds, Use:

.. code-block:: console

   $ west flash --domain hello_world

For more information about the system build please read the :ref:`sysbuild` documentation.

You should see the following message in the terminal:

.. code-block:: console

   *** Booting MCUboot v2.2.0-171-g8513be710e5e ***
   *** Using Zephyr OS build v4.2.0-6156-ged85ac9ffda9 ***
   I: Starting bootloader
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Primary image: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
   I: Secondary image: magic=unset, swap_type=0x1, copy_done=0x3, image_ok=0x3
   I: Boot source: none
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Image index: 0, Swap type: none
   I: Bootloader chainload address offset: 0x10000
   I: Image version: v0.0.0
   I: Jumping to the first image slot
   *** Booting Zephyr OS build v4.2.0-6156-ged85ac9ffda9 ***
   Hello World! ek_ra8p1/r7ka8p1kflcac/cm85

References
**********
- `EK-RA8P1 Website`_
- `RA8P1 MCU group Website`_

.. _EK-RA8P1 Website:
   https://www.renesas.com/en/design-resources/boards-kits/ek-ra8p1

.. _RA8P1 MCU group Website:
   https://www.renesas.com/en/products/microcontrollers-microprocessors/ra-cortex-m-mcus/ra8p1-1ghz-arm-cortex-m85-and-ethos-u55-npu-based-ai-microcontroller

.. _EK-RA8P1 - User's Manual:
   https://www.renesas.com/en/document/mat/ek-ra8p1-v1-users-manual

.. _RA8P1 Group User's Manual Hardware:
   https://www.renesas.com/en/document/mah/ra8p1-group-users-manual-hardware
