.. _tsnes_loopback_test:

Renesas TSN-ES MAC Loopback Test
#################################

Overview
********

This test suite exercises the Renesas TSN Endpoint Station (TSN-ES) Ethernet
controller in MAC loopback mode (:kconfig:option:`CONFIG_ETH_RENESAS_RCARSOC_TSNES_CONFIG_LOOPBACK`).
Frames sent from the network interface are looped back internally by the
controller and received again, without depending on R-Switch3 forwarding or
an external link partner. The suite covers:

- Device readiness and link/interface up status
- Single packet TX/RX data integrity with random payloads
- Burst traffic with sequence numbering to detect dropped or corrupted frames
- Variable packet sizes, from minimum to near-jumbo frames

Supported Boards
*****************
- rcar_ironhide_x5h/r8a78000/r52

Building and Running
*********************

This test requires the TSN-ES controller configured for internal loopback
and is run on real hardware.

.. code-block:: console

   west build -b rcar_ironhide_x5h/r8a78000/r52 tests/boards/renesas/rcar/x5h/tsnes_loopback
   west flash
