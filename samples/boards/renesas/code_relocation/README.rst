.. zephyr:code-sample:: code_relocation_nocopy
   :name: Code relocation nocopy

   Relocate code, data, or bss sections using a custom linker script.

Overview
********
A simple example that demonstrates how relocation of code, data or bss sections
using a custom linker script targeting the Renesas RX architecture.

The sample relocates the content of the ``ext_code.c`` file to external PSRAM
(copied from internal flash at startup) and executes it from there. The data
variable from ``ext_code.c`` is placed in internal RAM. The ``sram_code.c``
file is copied entirely to internal SRAM. All other code (e.g. ``main()``,
Zephyr kernel) stays in the internal flash.

EK-RX74M platform instructions
*******************************

The EK-RX74M board has 32 MB of external PSRAM connected via the MEMC
controller. It is mapped to 0x50000000.

To build and flash the application:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/code_relocation
   :board: ek_rx74m
   :goals: build flash
   :compact:

Execution output
****************

.. code-block:: console

  *** Booting Zephyr OS build vX.X.X  ***
  Address of main function 0xffe00xxx
  Address of function_in_ext_flash 0x50000000
  Address of var_ext_sram_data 0x00000xxx (10)
  Address of function_in_sram 0x00000xxx
  Address of var_sram_data 0x00000xxx (10)
  Hello World! ek_rx74m
