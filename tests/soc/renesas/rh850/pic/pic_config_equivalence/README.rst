PIC config equivalence test
###########################

Register-level test of the RH850 PIC driver. It runs on the target: after boot it reads the ADC
trigger registers of PIC20 and PIC21 and compares **every** one of them with the values that the
overlay of the board must produce. A register that is not in the expected tables of ``src/main.c``
must read 0, so the test also proves that the reset values are not changed by the driver.

``test_pic_device_is_ready``
   ``R_PIC_Open()`` succeeded (the PIC device is ready).

``test_adc_level1_sources``
   ``PIC20ADCKnTSELj`` / ``PIC2ADCJnTSELj`` and ``PIC21ADCKnTSELj`` (the source bits).

``test_adc_level2_taud_channels``
   ``PIC20ADTEN4`` / ``PIC21ADTEN4`` (the TAUD channel masks, which have no ADC index).

``test_adc_edges``
   ``EDGSEL``: one 2-bit field per scan group, written only to a register that has a source of the
   scan group.

The overlays of ``boards/`` use these routes:

* U2A: ``ADC0 SG0 RISING TAUD0_CH2`` and ``ADC0 SG1 BOTH`` with ``TAUD0_CH2`` and ``TAUD1_CH5``.
* U2B: ``ADC0 SG0 RISING TAUD0_CH2`` and ``ADC1 SG1 BOTH`` with ``TAUD0_CH2``, ``TAUD2_CH2`` and
  ``TAUD2_CH3``.
* U2C: ``ADC0 SG0 RISING TAUD0_CH2`` and ``ADC2 SG1 BOTH TAUD2_CH2``.

Not covered
***********

* The triangle-wave registers (``PIC1REG200`` to ``PIC1REG203``, ``PIC1TAUD0SEL``, ``PIC1REG210``
  to ``PIC1REG213``).
* Each trigger source individually.

Running the test
****************

.. code-block:: console

   west twister -T tests/soc/renesas/rh850/pic/pic_config_equivalence \
       -p pb_rh850u2b_468pin/r7f70254xafabg-c --device-testing --device-serial <port>
