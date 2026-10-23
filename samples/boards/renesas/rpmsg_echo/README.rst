.. zephyr:code-sample:: rpmsg_echo
   :name: R-Car RPMsg echo

   Communicate with Linux userspace and kernel-space RPMsg clients on separate Cortex-R52 (CR52)
   cores.

Overview
********

The sample has two build variants that share the same application source:

* The default variant runs on CR52 cluster 0 core 0 from ``sram0``, uses MFIS common channel 0,
  and communicates with the Linux userspace OpenAMP RPMsg sample.
* The ``kernelspace`` variant runs on CR52 cluster 0 core 1 from ``sram1``, uses MFIS common
  channel 1, and communicates with the Linux kernel RPMsg sample.

The Linux kernel allocates the RPMsg vrings and buffers dynamically. The kernel-space variant
maps the Linux default CMA pool from ``0xa2600000`` through ``0xbfffffff`` as non-cacheable
shared memory and maps the resource-table page at ``0x96650000`` separately. Linux DMA
allocations that fall outside this CMA pool are not accessible to this sample.

This is a one-shot sample. After one echo test, Zephyr destroys the RPMsg endpoint and does not
recreate it; reset the CR52 before running the Linux client again.

Building individual sysbuild domains
************************************

Build only the userspace domain with:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/rpmsg_echo
   :board: rcar_ironhide_x5h/r8a78000/r52
   :west-args: --sysbuild --domain rpmsg_echo
   :goals: build
   :compact:

Build only the kernel-space domain with:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/rpmsg_echo
   :board: rcar_ironhide_x5h/r8a78000/r52
   :west-args: --sysbuild --domain rpmsg_echo_kernelspace
   :goals: build
   :compact:

Sysbuild configures every enabled domain before building the selected domain. Set
``SB_CONFIG_RPMSG_ECHO_KERNELSPACE_IMAGE=n`` if the kernel-space domain must not be configured.

Building both variants with sysbuild
************************************

Build both images in one invocation with:

.. zephyr-app-commands::
   :zephyr-app: samples/boards/renesas/rpmsg_echo
   :board: rcar_ironhide_x5h/r8a78000/r52
   :west-args: --sysbuild
   :goals: build
   :compact:

The following build artifacts are used to run the sample:

* ``build/rpmsg_echo/zephyr/rpmsg_echo.srec``: userspace ROM Writer image for CR52 cluster 0
  core 0.
* ``build/rpmsg_echo_kernelspace/zephyr/rpmsg_echo_kernelspace_flash.srec``: kernel-space
  ROM Writer image for CR52 cluster 0 core 1, with the resource table omitted.
* ``build/rpmsg_echo_kernelspace/zephyr/rpmsg_echo_kernelspace.elf``: kernel-space ELF supplied
  to Linux remoteproc to describe the RPMsg resources.
