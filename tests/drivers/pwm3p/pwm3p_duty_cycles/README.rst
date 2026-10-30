PWM3P duty cycles test
#######################

This is a hardware bring-up test. The ztest assertions in
``src/main.c`` only check that the driver *accepts* distinct per-phase duty commands and a full
0%-100% sweep without error. They cannot, by themselves, prove that the physical duty cycle
measured on each phase's output actually matches the commanded value - that requires a human
observing the real outputs.

This is a different concern from ``tests/drivers/pwm/pwm3p_deadtime_safety``, which checks the
*gap* between a phase's high-side and low-side edges (deadtime), not the commanded duty *level*.

Manual procedure
*****************

1. Flash this test to the target board (e.g. ``pb_rh850u2a_516pin/r7f702300bebbc-c``).
2. Connect a logic analyzer (or oscilloscope) to the high-side pin of phase U, V, and W (see the
   ``pwm3p0_default`` pinctrl group in the board overlay for the exact PORT/pin of each phase).
3. Run ``test_distinct_duty_per_phase_is_accepted``: it commands U=10%, V=50%, W=90% and holds
   for 2 seconds. The console prints a ``DUTY_CYCLES: ... commanding U=... V=... W=...`` line
   immediately before the hold. Measure each phase's high-side pulse width as a percentage of
   the period and confirm it matches the printed target within measurement tolerance.
4. Run ``test_duty_sweep_matches_commanded_percentage``: it sweeps all 3 phases together through
   0%, 25%, 50%, 75%, 100% and holds each step for 1 second. Confirm every phase tracks the
   commanded percentage at every step, not just phase U.
5. The test passes only if the ztest assertions report PASS **and** steps 3-4 found no
   discrepancy between commanded and measured duty on any phase.

A green ztest result alone does not close this test item - it is necessary but not sufficient.
