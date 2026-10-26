RH850 SENT emulator feature tests
#############################

Scope
*****

This test package covers:

* pause pulse in variable and fixed message-length modes;
* valid and invalid Fast Channel CRC with recovery;
* recommended and legacy CRC builds;
* calibration pulse error recovery;
* diverse nibble values and multiple Fast Channel frames;
* a multi-channel wiring template for two TAUD slave outputs and five or six
  RSENT inputs.

It intentionally does not add new DMA coverage, timestamp coverage

Important shared-master limitation
**********************************

Two slave outputs may share one TAUD master only when they transmit the same
falling-edge-to-falling-edge period stream. They may drive two physical pins
and fan out to separate RSENT receiver groups, but cannot send two independent
SENT payloads simultaneously. Independent streams require two master channels.

Expected wiring
***************

Single output::

  TAUD0 slave ch1 -> SENT RX under test
  Board GND        -> emulator/board GND

Shared-master two-output fan-out::

  TAUD0 slave ch1 -> RSENT group A: SENT0, SENT1, SENT2
  TAUD0 slave ch2 -> RSENT group B: SENT2, SENT3, SENT13 (U2B10)

The normal ``pwm0_master``, ``pwm0_slave``, and ``pwm1_slave`` nodes must remain disabled when
the emulator owns the same TAUD channels.
