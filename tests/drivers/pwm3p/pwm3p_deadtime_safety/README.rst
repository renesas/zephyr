PWM3P deadtime safety test
###########################

This is a hardware bring-up test. The ztest
assertions in ``src/main.c`` only check that the driver *accepts* a large,
sudden duty-cycle jump (5% -> 95% -> 5%, repeated) on all 3 phases without
error. They cannot, by themselves, prove that the physical deadtime gap
between a phase's high-side and low-side output never reaches 0 - that
requires a human observing the real outputs.

Manual procedure
*****************

1. Flash this test to the target board (e.g. ``pb_rh850u2a_516pin/r7f702300bebbc-c``).
2. Connect a logic analyzer (or a 2-channel oscilloscope) to the high-side and
   low-side pins of phase U (see the ``pwm3p0_default`` pinctrl group in the board overlay
   for the exact PORT/pin). Repeat for phase V and phase W if possible.
3. Run the test and capture the waveform continuously while it executes -
   the console prints a ``DEADTIME_SAFETY: ... transition -> duty=...`` line
   immediately before each duty-cycle update, so the capture can be
   correlated with the exact transition under test.
4. For every high-side falling edge and every low-side rising edge (and vice
   versa), measure the gap between the two edges and confirm it is always
   **greater than or equal to** the ``deadtime-counts`` value configured for
   that node (converted to time via the PWM3P counter clock rate, see
   ``pwm3p_get_cycles_per_sec()``). Pay particular attention to the
   transitions immediately following a 5% -> 95% or 95% -> 5% jump, since
   that is the worst case for the duty-cycle update logic to get the
   deadtime insertion wrong.
5. The test passes only if the ztest assertions report PASS **and** step 4
   found no deadtime violation on any observed transition.

A green ztest result alone does not close this test item - it is necessary
but not sufficient.
