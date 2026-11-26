# Step-1 locomotion envelope: 2026-11-13, about 12:50 UTC

- **Evidence class:** physical robot telemetry.
- **Sequence:** `lite3_envelope_step1`, run under a one-hour authorization.
- **Confirmation:** the operator confirmed every safety item once. There were no
  prompts during the run.
- **Data:** `~/lite3_ws/log/movement_tests/20261113-125014-physical/` (gitignored).
- **Recorded verdict:** `FAILED_SAFE`. Every move passed. Only the silence-stop
  step failed (see below). The robot finished lying down and at rest, as the
  telemetry confirms.

## Locomotion response (vendor leg odometry)

| Move | Command | Expected | Measured | Ratio | Lateral m | Stopped (zeros) s |
| --- | --- | --- | --- | --- | --- | --- |
| fwd_005 | 0.05 m/s for 3 s | 0.150 m | 0.123 m | 0.82 | 0.001 | 1.91 |
| back_005 | −0.05 m/s for 3 s | −0.150 m | −0.187 m | 1.25 | 0.000 | 1.87 |
| fwd_008 | 0.08 m/s for 2.5 s | 0.200 m | 0.174 m | 0.87 | 0.008 | 1.94 |
| back_008 | −0.08 m/s for 2.5 s | −0.200 m | −0.240 m | 1.20 | −0.001 | 1.97 |
| fwd_010 | 0.10 m/s for 2 s | 0.200 m | 0.174 m | 0.87 | 0.009 | 1.22 |
| back_010 | −0.10 m/s for 2 s | −0.200 m | −0.245 m | 1.22 | −0.003 | 2.03 |
| turn_left_02 | +0.2 rad/s for 3 s | 0.600 rad | 0.607 rad | 1.01 | −0.013 | 1.22 |
| turn_right_02 | −0.2 rad/s for 3 s | −0.600 rad | −0.593 rad | 0.99 | 0.019 | 1.88 |
| silence_stop_005 | 0.05 m/s for 2 s, then silence | 0.100 m | 0.097 m | 0.97 | 0.009 | (none) |

"Stopped" is how long `stop_verify` took to confirm rest, including its 1.5 s
hold window. It starts after the move's own 2 s zero burst.

## Findings

- **Locomotion works across 0.05–0.10 m/s and ±0.2 rad/s.** Lateral drift was at
  most 1.9 cm per move and heading change at most 0.008 rad on the straight moves.
- **The response is asymmetric and repeatable.** Forward reaches 0.82–0.87 of
  the command; backward reaches 1.20–1.25. Turning is about 1.00. This is
  estimator-reported. A tape or video measurement is needed to tell whether the
  bias comes from the robot or from the odometry estimator.
- **The robot stops when commands go silent, in about 1.6 s.** After the last
  nonzero command, translation and leg stepping both ended about 1.6 s later.
  The robot travelled at most 3.4 cm in that time and settled at 2.5 cm. The
  firmware or bridge path therefore has a command-loss stop, which the vendor
  `motion_sender` source does not show. At Nav2's configured 0.5 m/s, 1.6 s
  would mean roughly 0.8 m of travel, so Nav2 speeds must stay capped.
- **The silence step's FAILED verdict came from a test-implementation flaw, not
  from the robot.** The step required the 1.5 s rest-confirmation window to
  *finish* within the 3 s timeout, which effectively demanded rest within 1.5 s.
  The fix: rest must *begin* within the timeout, then be confirmed by the hold
  window. A regression test now covers it. The recorded verdict is unchanged. A
  rerun under the corrected criterion would confirm the result formally.
- **Stand and lie-down transitions** detected again at 0.327 m and 0.081 m.
