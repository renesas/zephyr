.. zephyr:code-sample:: pwm3p
   :name: PWM3P (3-phase PWM)
   :relevant-api: pwm3p_interface

   Drive a 3-phase PWM (PWM3P) device and sweep its duty cycle.

Overview
********

This sample demonstrates the :ref:`PWM3P API <pwm3p_api>`: which models a single 3-phase PWM
device consisting of three pairs of complementary U/V/W high/low outputs that share a common period
and deadtime. It reads the configured period and counter clock rate of a PWM3P instance,
enables all six complementary outputs, and sweeps the shared duty cycle of the three phases
between 25% and 75% once per second.

Requirements
************

This sample requires a board with a Renesas RH850 TAUD3P (PWM3P) instance,
for example ``pb_rh850u2a_516pin``, ``pb_rh850u2b_468pin``, or
``pb_rh850u2c_292pin``. The board overlays of the sample enable the ``taud3p0``
and ``pic0`` nodes, which are disabled by default.

Building and Running
*********************

.. zephyr-app-commands::
   :zephyr-app: samples/drivers/pwm3p
   :board: pb_rh850u2a_516pin/r7f702300bebbc-c
   :goals: build flash

Sample Output
=============

.. code-block:: console

   PWM3P ready: period=4000 counts, counter clock=80000000 Hz (PWM freq=20000 Hz)
   Cycle-end callback registered
   PWM3P output enabled, sweeping duty 25%..75%
   duty=25% (1000 counts), cycle_count=5
   duty=30% (1200 counts), cycle_count=25005
   ...

   <repeats endlessly>
