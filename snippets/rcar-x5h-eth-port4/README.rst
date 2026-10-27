.. _snippet-rcar-x5h-eth-port4:

R-Car Ironhide X5H Ethernet port4 Snippet (rcar-x5h-eth-port4)
##############################################################

.. code-block:: console

   west build -S rcar-x5h-eth-port4 [...]

Overview
********

This snippet makes R-Switch 3 port4, with the Q32xx 2.5GBASE-T1 PHY (``phy1``, USXGMII
through ``pcs_ch4``), the network interface of a networking application on
``rcar_ironhide_x5h/r8a78000/r52``. The port is served by GWCA0. Port5, which the
application overlay may have chosen instead, is disabled.

Requirements
************

Board: ``rcar_ironhide_x5h/r8a78000/r52``. The snippet only changes the devicetree; ring
sizes and other application settings stay in the application overlay.
