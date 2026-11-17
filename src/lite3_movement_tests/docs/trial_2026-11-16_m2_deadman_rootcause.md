# M2 dead-man stop failure: root cause (physical session 2026-11-16)

**Reports:** four M2 runs under `log/operator/run-20261116-113953/physical/`:
114257, 114334, 114452 and 114641. All four FAILED with "robot not at rest
within 3.0 s of the release".

A fifth attempt (114627) aborted during `stand` because joint telemetry was
stale. That was a separate link stall.

**Analysis basis:** recorded 20 Hz traces (leg odometry, leg extensions),
robot-side command counts, and the step and runner code. Times are seconds after
the nominal dead-man release, which is 1.0 s after motion start.

## 1. Timeline (reproduced in all four runs)

| Event | 114257 | 114334 | 114452 | 114641 |
| --- | --- | --- | --- | --- |
| Last command reaching `motion_sender` (dead-man cut) | 0.41 | 0.46 | 0.41 | 0.42 |
| Forward translation ends (vx < 0.02) | 1.08 | 1.14 | 1.06 | 1.02 |
| Leg stepping ends (leg-extension change < 5 mm) | 2.19 | 2.18 | 2.19 | 1.91 |
| Post-gait settling: yaw-rate peak above 0.05 rad/s | 3.30 (0.072) | 3.35 (0.088) | 3.32 (0.055) | 3.09 (0.060) |
| First full 1.5 s quiet window begins (rest onset) | 3.38 | 3.45 | > 3.0 | > 3.0 |

For the last two runs, the observation ended at 4.5 s, before a later onset
could be confirmed.

Run 114334 in detail:
- **Motion start:** t = 2.05 s session time.
- **Release:** at 3.05 s.
- **Robot-side commands:** 20 non-zero before the release, 9 after, the last at
  3.51 s. The dead-man lock's 0.5 s timeout accounts for those 9.
- **Speed:** forward speed 0.04–0.06 m/s until about 4.1 s, then a brief reverse
  and yaw oscillation while the gait winds down.
- **Joints:** frozen from 5.30 s.
- **Settling:** at 6.35–6.60 s the body makes one settling motion (yaw −0.006
  rad, peak −0.087 rad/s; leg extensions 0.319 → 0.313 m).
- **After that:** fully quiet.

## 2. Did the robot exceed 3 s, or did verification miss an earlier stop?

**Neither "false failure" nor a long coast.** The robot stopped *walking* well
within 3 s: translation ended after about 1.1 s and stepping after about
2.2 s. About 1.1 s after stepping ends, the Lite3 makes a small posture-settling
movement. This movement is real body motion, not sensor noise.

The rest criterion requires all of joint range, odometry speed, yaw rate,
position change and gyro to be quiet for a full 1.5 s window. Under that
definition, rest begins only after the settling movement, at about 3.4–3.5 s.
The 3 s requirement, as currently defined, was therefore genuinely not met. The
verification logic evaluated it correctly.

The same settling movement follows the zero-commanded stops of the M1 run that
passed. There it came about 2.3 s after the commands ended, about 1.1 s after the
legs stopped, and stayed below the 3 s limit because zeros end the gait about
0.5 s sooner than silence does. **It is a property of the Lite3's
gait-to-stand transition, not of the dead-man path.**

The dead-man path itself performed as designed:
- Commands were cut at 0.41–0.46 s (limit 0.8 s).
- The robot coasted at most about 5 cm.

The 3 s value was set when M2 was designed (2026-11-16). It was not derived from
measured Lite3 stop dynamics. The envelope trial had measured stepping ending
about 1.6 s after silence, but not the settling movement that follows.

## 3. Why the step took 7.5 s

The design produces 7.5 s; no deadline was overrun:

| Phase | Duration |
| --- | --- |
| Command phase. The test keeps publishing for the full 3.0 s, and the dead-man is released at 1.0 s so the lock is tested against live commands. | 3.02 s |
| Observation until `stop_within_s` + `hold_s` = 3.0 + 1.5 s after release. Rest must *begin* by 3 s and is confirmed by a full 1.5 s window, so the loop cannot decide earlier. | about 2.5 s |
| Closing zero burst (`zero_seconds`) | 2.0 s |

The 3 s limit applies to the rest *onset*, not to the step's duration.

## 4. Noise, timestamps, freshness

- **Not sensor noise.** The settling peak (0.055–0.088 rad/s) shows up in yaw and
  leg extensions together, at the same moment relative to the end of stepping in
  every run. At-rest noise is ≤ 0.005 rad/s.
- **Not timestamp synchronisation.** Every window uses local receipt time.
- **Not telemetry freshness.** No staleness abort occurred during M2. Trace gaps
  were ≤ 0.15 s.
- **Not the safe stop contradicting the step.** The runner's `safe_stop` began
  6.5 s after release, after the step's own 2 s zero burst. The robot had been
  quiet since about 3.5 s, so rest was verified immediately with no send errors.

## 5. The 0.064 m displacement

Forward along the start heading (run 114334):
- 0.015 m at release.
- Maximum 0.071 m about 1.2 s after release.
- 0.064 m at the end, after a 7 mm backward settle.

**About 0.05 m happened after the release.** That breaks down as about 0.46 s of
commands still reaching the robot inside the dead-man timeout, plus about 0.6 s
of deceleration. The robot had only just reached speed: the legs start moving
about 0.15 s after the first command, but forward speed builds up only after
about 0.6 s, with a brief backward/sideways shuffle first. That is why only
0.015 m was covered in the first second.

## 6. Logging gaps found and fixed (validated without actuation)

**What the old reports were missing:**
- Only the final rest reasons.
- No gyro and no raw joint motion in the trace.
- No command or dead-man timestamps.

**What the new reports record, for every M2:**
- **Rest timeline:** `rest_timeline()` re-evaluates every rest criterion at
  10 Hz over the whole step, using the instantaneous value plus the 1.5 s window
  verdict and its failing reasons. `settle_summary()` gives, per criterion, its
  peak and the last time it exceeded its limit.
- **Timing:**
  - Phase times: commands cut, translation ended, joint motion ended, yaw motion
    ended, last disturbance, rest onset.
  - The last dead-man feed and first/last robot-side command times, relative to
    release.
  - Forward displacement at release, at its maximum, and at the end.
  - An explanation of the step's duration.
- **Readable failure:** the failure summary names the criterion that settled
  last. `report.md` gets a "Stop diagnostics" section with a per-criterion table
  and a 0.5 s timeline.
- **Every trace:** rows now include `gyro` (max |rate|) and `jd` (max raw joint
  change since the previous row).

**Validation (software only):**
- A new simulator fault, `settle_twitch`, reproduces the physical signature:
  about 1.2 s hold after silence, then a settling motion 1.1 s after stepping
  ends.
- M2 still FAILS under it with the 3 s limit unchanged.
- The diagnostics report commands cut at 0.5 s, translation ended at 1.9 s, and
  yaw and joint motion last at 3.7 s.
- Nominal M2 still passes.
- 99 tests pass. Nothing was sent to the robot.

## 7. Decision needed before rerunning M2 (limits not changed)

The 3 s requirement and all interlocks are unchanged. M2 as defined will keep
failing on this robot until one of the following is chosen:

1. **Define the requirement precisely.** "Locomotion stopped" means no
   translation, no stepping, and position held within 3 s. A separate,
   explicitly limited "fully settled" criterion would cover the posture settle,
   for example ≤ 5 s with a yaw-change bound. Everything stays measured. The
   safety-relevant quantity, how far and how long the robot keeps moving, is
   what the 3 s guards.
2. **Keep full quiescence within 3 s,** and accept that the Lite3 cannot meet it
   after a lock-induced silence stop (about 3.4–3.5 s measured). This would
   block M3 and later tests.
3. **Make the lock command zeros instead of silence.** For example, a
   locked-state zero publisher at the mux. Zeros ended the gait about 0.5 s
   sooner in M1. That gives an estimated onset of about 2.9–3.0 s: marginal, and
   it needs its own validation.

A separate reporting point: the step marks the state "uncertain" when rest is
not confirmed. The runner keeps that flag even though its safe stop then
verified rest two seconds later. That is why the verdict was
INTERVENTION_REQUIRED instead of FAILED_SAFE. Whether a verified safe stop should
clear the flag is a policy choice and has not been changed.

## 8. Decision and change (2026-11-16): option 1, everything measured

**M2 now judges two measured stages.** Both are confirmed by a full 1.5 s window,
judged on onset, and timed from the release:

1. **Stopped moving within 3 s.** Hard ceiling 3 s; previously configurable up
   to 4 s. "Stopped moving" means, over the whole window:
   - odometry speed ≤ 0.04 m/s;
   - position change ≤ 0.02 m;
   - **leg-extension range ≤ 0.02 m (no foot lifts).**

   The leg-extension threshold comes from the recorded data. Steps change leg
   extension by 0.05–0.06 m; the post-gait settle changes it by ≤ 0.008 m.
2. **Fully settled within 5 s.** Hard ceiling 6 s. This uses the unchanged full
   rest criteria, and the motion between the stop onset and the settled pose is
   bounded:
   - heading ≤ 0.05 rad;
   - position ≤ 0.02 m.

**Unchanged:**
- The command cut-off limit (0.8 s).
- The rule that the robot must move before the release.
- Every interlock, and the full rest criteria used everywhere else (`stop_verify`,
  safe stop).
- A robot still walking after 3 s fails M2. The state is then marked uncertain,
  and the runner commands zeros.

**Against the four recorded runs** (estimated from their 20 Hz traces):
- Stopped moving at 2.14, 2.18, 2.19 and 1.86 s.
- Settle turn 0.0025–0.0055 rad; settle shift ≤ 0.9 mm.
- Full rest began at about 3.4–3.5 s where the old observation window covered
  it.

**Simulation:**
- The `settle_twitch` signature now passes.
- Still failing, as they must:
  - a robot that keeps walking after silence (`no_silence_stop`);
  - an over-large settle rotation (`settle_twitch_large`);
  - any attempt to configure `stop_within_s` above 3 s.

**Gating:** M3 now needs only M0, and M4 needs M2 and M3.
