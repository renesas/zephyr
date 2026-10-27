.. _snippet-rcar-x5h-eth-port5:

R-Car Ironhide X5H Ethernet port5 Snippet (rcar-x5h-eth-port5)
##############################################################

.. code-block:: console

   west build -S rcar-x5h-eth-port5 [...]

Overview
********

This snippet makes R-Switch 3 port5, with the DP83869 1000BASE-T PHY (``phy2``, SGMII
through ``pcs_ch5``), the network interface of a networking application on
``rcar_ironhide_x5h/r8a78000/r52``. The port is served by GWCA0. Port4, which the
application overlay may have chosen instead, is disabled.

Requirements
************

Board: ``rcar_ironhide_x5h/r8a78000/r52``. The snippet only changes the devicetree; ring
sizes and other application settings stay in the application overlay.
