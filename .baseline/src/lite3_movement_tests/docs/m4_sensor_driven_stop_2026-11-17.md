# M4 redesign: continuous sensor-driven proximity stop (2026-11-17)

Status: implemented and validated in software. **Not yet run on the robot.**

## 1. Why

The old M4 had two hardcoded constraints:
- the obstacle had to start within ±0.05 m of `boundary + 0.065` m;
- the walk could last at most 3 s.

On 2026-11-16, **12 of 14 physical M4 attempts never moved**, refused by
`detect_approach_window` because the chair was at 0.12–1.32 m instead of the window. The 2 runs
that passed used the older terrain-lock version.

## 2. What M4 is now

The robot walks straight ahead with **no goal and no obstacle position**. It continues while
fresh LiDAR data shows the path is clear. The protective stop must stop it when an obstacle
reaches the boundary.

`READY → WALKING → OBSTACLE_DETECTED → STOPPING → STOP_VERIFIED`

Code:
- `lite3_movement_tests/m4_steps.py`: the step, the shadow and the evaluation.
- `lite3_movement_tests/proximity.py`: the tracker and outcomes. Pure Python, unit-tested.
- `config.py`: limits.
- `physical_suite.py`: CLI, profiles, measured latency.

### Components: reused, none duplicated

| Function | Component |
| --- | --- |
| Forward command | runner → `/lite3_ops/test_cmd_vel` (twist_mux prio 20). This is the existing path used by M1–M3 |
| Dead-man / backup lock | twist_mux `operator_hold` / `terrain_stop` (unchanged) |
| **Protective stop** | **Nav2 `collision_monitor`** STOP polygon. It is the only `/cmd_vel` publisher; the boundary is computed by `collision_profile` |
| Detection | `terrain_monitor` forward corridor (±0.30 m, 3 m from the centre), i.e. LiDAR |
| Actuation | `lite3_domain_bridge_8` → `motion_sender` (unchanged) |

**Is collision_monitor the right primary?** Yes. It was already the final command authority, and
on the real binary (`tools/check_collision_monitor.py`, new approach scenario):
- the first zero came **1 ms** after the cloud that put the obstacle inside the polygon;
- there were no early zeros and no forward commands after the stop.

Its known Humble gap is that it ignores stale sources. That stays covered by the terrain lock
(0.5 s) and by the runner's own freshness checks. No custom stop controller was added. The runner
only commands forward or zero through the existing inputs.

### Stopping boundary

`collision_profile.boundary()` = reaction travel + coast + sensor margin + clearance:
- **Reaction travel** = speed × (live LiDAR age p95 + processing + command-path latency).
- **Command-path latency** is now **measured**. M2 records the first-command delivery time, and
  the allowance is that time + 0.05 s for the collision_monitor cycle, at least 0.10 s. It is
  0.15 s until measured. It is also saved to `calibration.json` for the next start-up.
- **Coast** is measured by M2 on the same navigation run.

M4 refuses unless the running polygon:
- covers this value; **and**
- is at least 0.05 m beyond the 0.30 m backup lock.

The second rule exists because a 0.31 m boundary (the default `--clearance 0.15`) fires on the
same scan as the backup lock, which would make the PASS attribution a coin toss. Use
`start_navigation.sh --clearance 0.20` (boundary 0.36 m with the current calibration).

Live read-only measurement, 2026-11-17 11:17:

| Quantity | Value |
| --- | --- |
| LiDAR rate | 10 Hz |
| Maximum status gap | 0.12 s |
| Sensing age p95 | 0.18 s |
| Boundary required at 0.05 m/s for 0.15 m clearance | 0.254 m |
| Only blocker | 0.31 m boundary vs 0.30 m backup lock |

### PASS and other outcomes

Only `STOPPED_FOR_OBSTACLE` is a PASS. It needs all of the following:
- the robot moved;
- the obstacle was confirmed and tracked before the stop;
- the collision_monitor stopped the robot for **that** obstacle (within boundary + 0.05 m);
- the runner kept publishing forward for the suppression window and the robot saw zeros;
- the response from boundary crossing to the last forward command at the robot was
  ≤ min(`--response-limit`, latency + 0.05 s);
- the measured command latency was ≤ the latency the boundary assumes;
- the robot stopped moving within 3 s;
- minimum and final clearance were ≥ `--min-clearance`.

Separate outcomes:
- `STOP_CRITERIA_FAILED`
- `TIME_LIMIT`
- `TRAVEL_LIMIT`
- `BACKUP_LOCK_STOP`
- `UNEXPLAINED_STOP` (dead-man or command path)
- `OPERATOR_STOP`
- `SAFETY_ABORT`

### Safeguards (independent of whether the collision_monitor works)

Freshness and rates of terrain, hazard cloud and lock input. IMU and joint validity on every tick.
Then the tracker:
- An obstacle visible at the start is tracked from step one.
- **Lost tracking** aborts.
- So does any **inconsistency with the robot's own travel**, allowing up to 25 % odometry
  under-read; this catches moving objects or something stepping in.

Further checks:
- **Proximity-lock failure**: the obstacle is inside 0.30 m but the lock input is false.
- **Protective-stop failure**: the obstacle is inside the boundary, yet forward commands still
  reach the robot after `--response-limit`. The runner then stops the robot itself.
- Hard floor at 0.10 m.
- **Odometry budget** to the *last consistent* obstacle measurement. It is re-anchored per scan;
  anchoring only at first detection would wrongly stop long approaches beyond about 1.3 m.
- Lateral, yaw, backward and over-speed motion; robot-side command speed above the
  authorisation; scheduler gap.
- Travel: odometry ÷ 0.8 **and** SLAM pose. Duration. Odometry geofence = travel + 0.2 m.

New hard limits (refused, never clamped):

| Limit | Value |
| --- | --- |
| Speed | ≤ 0.05 m/s (unchanged) |
| Travel | ≤ 1.8 m |
| Duration | ≤ 60 s (was 20 s with a 3 s move) |
| Detection range | ≤ 2.5 m |
| Sensor age | ≤ 0.5 s (only tighter) |
| LiDAR rate | ≥ 8 Hz (only higher) |
| Response | ≤ 0.5 s |

Travel and duration never change the speed, the boundary or any safeguard (test
`test_travel_and_duration_never_change_speed_or_stopping`). The authorization now also binds
M4 travel.

### Residual risk

A LiDAR that goes blind while still reporting healthy "nothing ahead" data, with the obstacle
not visible at the start, defeats every sensing layer, because they share the sensor. What
remains is the travel and time limits and the operator's e-stop. Mitigation in the procedure:
for the first sessions, place the obstacle 0.8–1.2 m ahead so that it is tracked from step one.

## 3. M5 overspeed: root cause and fix

**Evidence** (run-20261116-131140):
- Five M5 runs aborted on "robot-side command beyond authorized limits": vx 0.063–0.098 against
  caps of 0.05 and 0.08.
- Nav2's own `/nav_cmd_vel` already exceeded the cap, ramping 0.0175 → 0.0692 at the smoother's
  0.5 m/s² limit. So the runner's tolerance was not the problem.
- In every session, `launch.log` shows `controller_server: Optimizer reset` 2–4 ms after
  `Received a goal`.
- The one M5 that passed used `--max-speed 0.10`, which equals MPPI's static `vx_max`.

**Cause**: in Humble 1.1.20, `mppi::Optimizer::reset()` restores the base constraints. A
`/speed_limit` published *before* the goal, as the runner did, is discarded when the goal
arrives.

**Fix** (`nav_steps.step_nav_goal`, `ros_io.NavRobot.set_nav_velocity_cap`):
1. Before each goal, set the velocity_smoother's dynamic `max_velocity` to
   `[max_speed, 0, 0.20]` and read it back. If the cap cannot be verified, no goal is sent. The
   smoother sits after MPPI, clamps every output and is not reset per goal.
2. Re-send `/speed_limit` after the goal is accepted and every 1 s, so that MPPI plans at the
   limit instead of saturating.

The robot-side cap check is unchanged.

**Validation**:
- `tools/check_velocity_cap.py` on the real Humble velocity_smoother: parameter accepted, read
  back as `[0.05, 0, 0.2]`, 0.10 m/s in → 0.05 out.
- The simulator now models the goal-start reset:
  - without either fix, M5 reproduces the physical abort;
  - each half of the fix alone holds the limit;
  - an unverifiable cap refuses the goal.

**M4 is not exposed to this**: Nav2 has no goal during M4, any Nav2 output during M4 aborts, and
robot-side speed is checked against M4's 0.05 m/s authorisation.

## 4. M2 and M3 prerequisites

- **M2 dead-man**: resolved 2026-11-16 by the two-stage criterion
  (`trial_2026-11-16_m2_deadman_rootcause.md`). Passed 13:17 and 13:57 on 2026-11-16, and
  10:45 on 2026-11-17.
- **M3 leak**: resolved by re-confirming the lock before each command
  (`trial_2026-11-16_m3_lock_release.md`). Passed 13:15 and 13:58 on 2026-11-16.

Physical M4 still requires M2 and M3 to pass **on the same navigation run**. Today's run has M0,
M1 and M2, but it was started with `--clearance 0.15`, so it must be restarted anyway.

## 5. Validation summary (no actuation)

| Check | Result |
| --- | --- |
| `lite3_movement_tests` pytest | 192 passed (126 before). `test/test_m4.py` adds 67 tests |
| `lite3_locomotion` pytest | 202 passed |
| Real collision_monitor (isolated domain) | PASS, incl. M4 approach: 1 ms response |
| Real velocity_smoother cap (isolated domain) | PASS |
| Real twist_mux dead-man / test / stop inputs (isolated domain) | PASS |
| `run_physical.sh --sim` M0–M5 at 0.05 m/s, boundary 0.36 | 6 × PASS |
| `--sim` M4, obstacle at 0.7 / 1.2 / 1.7 m and with the first-session profile | `STOPPED_FOR_OBSTACLE` |
| `--sim` M4, no obstacle | `TRAVEL_LIMIT` (not PASS) |
| Live read-only M4 READY on the armed stack | All checks pass except the boundary separation (expected) |
| Live read-only shadow | Sensors OK at 9–11 Hz. Object at 2.42 m correctly outside the 2.0 m region |
