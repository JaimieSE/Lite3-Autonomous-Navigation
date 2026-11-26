# Lite3 supervised physical tests (M0–M9)

These tests **move the robot**. They build up step by step: from a command-path
check, through basic walking and stop verification, to obstacle stops, Nav2
navigation and a full course. Everything runs on the **group8 container**
(`ssh user@192.168.10.78`, `cd ~/lite3_ws`). The day-to-day dry workflow
(preflight, Test 4) is in [README.md](README.md).

> **You are the emergency stop.** Before every physical test, hold the handheld
> remote or e-stop and check that it works. No software here replaces it. Ctrl+C
> and `operator/abort_physical.sh` command a stop, but they are **not**
> emergency stops.

## 1. How it is built (nothing new talks to the robot)

```text
Nav2 (university launch, Lite3 caps) -> /nav_cmd_vel ------- priority 10 -+
test runner, bounded test moves      -> /lite3_ops/test_cmd_vel  prio 20 -+-> twist_mux -> /cmd_vel
test runner, zero-velocity stops     -> /lite3_ops/stop_cmd_vel prio 255 -+       |
                                                                                  v
  locks (block priority < 50):                         lite3_domain_bridge_8 -> domain 64
    terrain_stop  = LiDAR proximity stop                 -> motion_sender (vendor) -> robot
    operator_hold = DEAD-MAN, held only by a running, healthy test
```

- **Dry is the default everywhere.** Physical mode needs two separate arming steps:
  1. `operator/start_navigation.sh --physical`. You type `ARM`; the script refuses
     degraded LiDAR.
  2. `operator/run_physical.sh --physical`. You type the test ID (for example
     `M5`) before **each** test.
- **Dead-man lock.** In physical mode, Nav2 cannot move the robot unless a test
  is running and all of its checks pass: about 20 times a second, the test tells
  twist_mux "not on hold". If the test stops, crashes, hangs, or your SSH session
  drops, the lock engages within 0.5 s, so Nav2's commands stop reaching the robot.
  An idle physical stack is locked. This was verified on the real twist_mux binary
  with `src/lite3_locomotion/tools/check_mux_lock.py --physical`.
- **When a lock engages**, twist_mux goes silent and the robot coasts to a stop
  in about 1.6 s (measured 2026-11-13). When the test itself stops the robot, it
  commands zeros on the priority-255 input instead, which passes every lock.
- **The test aborts at once and stops the robot if any of these happen:**
  - Body tilt is beyond 0.35 rad.
  - Telemetry is older than 0.25 s.
  - Someone moves the handheld remote (takeover).
  - The robot leaves the geofence.
  - A command reaching `motion_sender` is faster than the authorised speed, or
    is a reverse command during navigation.
  - Another program publishes on `/cmd_vel`, the test inputs, or the joystick
    input.
  - Nav2 commands motion outside a navigation step.
  - An obstacle gets within 0.08 m of the footprint while navigating.
  - The hazard feed goes stale.
  - The session runs past 10 minutes.
- **Hard limits, set in code** (`lite3_movement_tests/config.py`). No terminal
  flag can exceed them; out-of-range values are refused, not clamped:
  - Speed **0.5 m/s** (raised from 0.10 on 2026-11-17; **only up to 0.10 m/s is validated**, see §1.1). M4's approach: 0.05 m/s.
  - Yaw rate 0.20 rad/s under Nav2.
  - Direct moves 3 s.
  - Goals within 6 m, geofence within 10 m.
  - Proximity stop no closer than **0.30 m from the nose**. The terrain monitor
    itself also refuses lower values.

### 1.1 Speeds above 0.10 m/s (hard ceiling 0.5 m/s)

The ceiling was raised to 0.5 m/s by the operator's decision. **Everything measured so far is at or below
0.10 m/s** (coast 0.036 m at 0.05 m/s, the 0.30 m backup lock, the Nav2 tuning). Above that:

- The default stays 0.10 m/s. Higher speeds are opt-in: `operator/start_navigation.sh --physical
  --max-speed 0.3 --clearance C`. Tests and roaming refuse a speed above the running stack's `MAX_SPEED`.
- The stopping boundary grows quickly. Braking distance beyond 0.10 m/s is **assumed** to scale with
  speed^1.5 (the truth lies between linear and quadratic). It is conservative, not measured. With today's calibration
  and 0.15 m clearance the boundary is 0.32 m at 0.10 m/s, 0.50 m at 0.2, 0.71 m at 0.3, 0.96 m at 0.4 and
  1.23 m at 0.5 (cap 1.5 m). The start-up prints it.
- The 0.30 m **backup lock cannot act as a backup**: the robot would travel past it before stopping. M10
  moves it to 5 cm inside the boundary (and restores it afterwards). `run_physical.sh` refuses
  `--max-speed` above 0.10 unless `--stop-distance` is at least boundary - 0.05.
- Before using a speed above 0.10, run **M2 at that speed** (`--max-speed V`) on the same navigation run:
  it measures the real coast, replacing the assumption, and gives the next start-up a valid boundary.
- M1/M2 use a 0.5 m geofence and short moves; M4's approach speed stays capped at 0.05 m/s.
- The robot-side overspeed check uses the session's authorised speed. Without an authorization (simulation)
  it can only use the 0.5 m/s ceiling.

## 2. Readiness checklist (before the first physical session)

Run these in order. Each must be true.

| # | Check | How |
| --- | --- | --- |
| 1 | Current software built (the terrain monitor needs the stop-distance parameter) | `operator/build.sh` |
| 2 | LiDAR link healthy: preflight **READY**, not DEGRADED | `operator/preflight.sh`. If reassembly failures are reported, apply README §8.1 (needs your sudo). |
| 3 | Rehearse the tests in simulation | `operator/run_physical.sh --sim --phases M0,M1,M2,M3,M4,M5` should show 6 × PASS. This proves the software only, not the robot. |
| 4 | Robot powered on, **Navigation (auto) mode** selected with the handheld remote/app (lab procedure), on flat ground with space | Physically. Telemetry cannot show the mode; M1 checks it by behaviour (no response = FAIL). |
| 5 | Remote / e-stop tested today, in your hand | Physically |
| 6 | Exclusive use: no other group commanding this robot | Ask in the lab |
| 7 | Dry readiness on the armed stack | After `start_navigation.sh --physical`: `operator/run_physical.sh --phases M0`. Without `--physical`, nothing is published; the run lists every blocker. |

## 3. Procedure

```bash
operator/stop_navigation.sh                 # if the dry stack is running
operator/start_navigation.sh --physical --clearance 0.20   # type ARM; waits for READY (dead-man LOCKED)
                                            # --clearance 0.20 -> stop boundary 0.36 m; M4 needs >= 0.35 m (§3.1)
operator/status.sh                          # PHYSICAL checks: only twist_mux on /cmd_vel, locks wired

operator/run_physical.sh --physical --phases M0,M1,M2   # path, basic motion, stop paths
operator/run_physical.sh --phases DET --object-distance 0.20   # dry: is the box seen steadily?
operator/run_physical.sh --physical --phases M3         # proximity lock blocks motion (needs only M0)
operator/run_physical.sh --phases M4                    # dry: M4 READY check, nothing published
operator/run_physical.sh --phases M4 --shadow           # dry: slide the chair toward the robot (§3.1)
operator/run_physical.sh --physical --phases M4 --profile operator/profiles/m4_first_session.yaml
                                                        # walk until the chair stops it (needs M2, M3)
operator/run_physical.sh --physical --phases M5         # Nav2 straight line
operator/run_physical.sh --physical --phases M6         # corner / junction
operator/run_physical.sh --physical --phases M7         # avoidance (box on the line)
operator/run_physical.sh --physical --phases M8         # blocked goal, recoveries
operator/run_physical.sh --physical --phases M9 --waypoints "1.5,0;2.5,1.0,90;2.5,2.5"

operator/run_physical.sh --physical --phases SIT        # lie down at the end
operator/stop_navigation.sh                             # always
```

Each test runs like this:

1. It prints the setup, the PASS criteria, the limits and the steps (`*` marks
   steps that send commands).
2. It asks you to confirm the checklist by typing the test ID.
3. It runs without further prompts, printing a live line once a second while
   Nav2 drives.

### 3.1 M4: walk until a LiDAR-detected obstacle stops the robot

M4 has **no goal and no start position for the obstacle**. The robot walks straight ahead at a
low, bounded speed. It keeps walking only while fresh LiDAR data shows the path is clear. Nav2's
`collision_monitor` must stop it when an obstacle reaches the stopping boundary, wherever the
obstacle was placed or appeared.

```text
READY -> WALKING -> OBSTACLE_DETECTED -> STOPPING -> STOP_VERIFIED
```

| State | What happens |
| --- | --- |
| READY | Standing still. Telemetry, LiDAR rate and gaps, terrain monitor, hazard cloud, lock input and command path are healthy. The boundary is re-checked (below). Nothing is in the path closer than `--initial-clear` (default boundary + 0.15 m). |
| WALKING | Forward commands at `--approach-speed`, 20 per second, only while **every** check passes. The dead-man is fed only then. |
| OBSTACLE_DETECTED | The nearest object in the forward corridor (±0.30 m, up to `--detection-range`) is seen in `--detect-scans` consistent scans. From then on it is tracked against leg odometry. An object already visible at the start is tracked from the first step. |
| STOPPING | The `collision_monitor` turns the forward commands into zeros at the robot. The test keeps publishing forward for `--suppression-proof` s, but only while it still sees the obstacle inside the boundary. Then it commands zeros on the priority-255 stop input. |
| STOP_VERIFIED | The robot has stopped moving: no translation, no foot lifts. Final clearance, stopping distance and latencies are recorded. |

**The stopping boundary** is the `collision_monitor` stop polygon. It is fixed when navigation
starts, from `start_navigation.sh --clearance`. Before moving, M4 checks that it covers what this
walk needs, from **live and measured** values:

| Term | Source | Value on 2026-11-17 (live, read-only) |
| --- | --- | --- |
| Reaction travel | speed × (LiDAR data age p95 measured now + terrain processing + command-path latency) | 0.05 × 0.353 s = 0.018 m |
| Command-path latency | first command → `motion_sender`, measured by this run's M2, + 0.05 s for the collision_monitor cycle (at least 0.10 s; 0.15 s until measured) | 0.15 s (not yet measured) |
| Coast after the command cut | measured by the **M2 that passed on this navigation run** (scaled up if faster) | 0.036 m |
| Sensor margin | one 5 cm terrain-grid cell | 0.050 m |
| Minimum clearance | `--min-clearance` (default: navigation `--clearance`) | 0.150 m |
| **Required** | sum | **0.254 m** |

It also requires the boundary to be **at least 0.05 m beyond the 0.30 m backup lock**. Otherwise
the backup lock and the primary stop fire on the same scan, and the test cannot show which layer
stopped the robot. With the default `--clearance 0.15` the boundary is 0.31 m, so **M4 refuses
and tells you the clearance to use**. Start navigation with `--clearance 0.20` (boundary
0.36 m). With `--clearance 0.19` the boundary is exactly 0.35 m, the minimum.

M4 refuses before moving in these cases:
- no M2 passed on this navigation run;
- the boundary is too small for the live delay or the coast;
- the boundary is too close to the backup lock;
- the path is not clear;
- the LiDAR is slower than `--min-lidar-hz`;
- data is older than `--max-sensor-age`.

**Outcomes** (printed as `M4 OUTCOME:` with a checklist; also in `report.json`). Only the first
is a PASS:

| Outcome | Meaning |
| --- | --- |
| `STOPPED_FOR_OBSTACLE` | PASS. All of these held: the robot moved ≥ 0.05 m; the obstacle was detected and tracked before the stop; the collision_monitor stopped the robot **for that obstacle** (within boundary + 0.05 m); forward commands kept being published and were zeroed at the robot (≥ 5 published, ≥ 3 robot-side zeros); forward commands stopped reaching the robot within min(`--response-limit`, command-path latency + 0.05 s) of the obstacle crossing the boundary; measured command latency ≤ the latency the boundary assumes; the robot stopped moving within 3 s; minimum and final clearance ≥ `--min-clearance`. |
| `STOP_CRITERIA_FAILED` | The collision_monitor stopped the robot for the obstacle, but a criterion above was missed. The checklist shows which one. |
| `TIME_LIMIT` / `TRAVEL_LIMIT` | The movement timer or the travel limit ended the walk. No protective stop happened. The summary says whether an obstacle was tracked. |
| `BACKUP_LOCK_STOP` | The terrain backup lock stopped the robot first, for example because someone came within 0.45 m beside it. |
| `UNEXPLAINED_STOP` | Commands stopped reaching the robot without a proximity stop: the dead-man lapsed, or the command path failed. |
| `OPERATOR_STOP` | Ctrl+C, `abort_physical.sh` or the handheld remote. |
| `SAFETY_ABORT` | A safeguard below stopped the robot. The reason is in the summary. |

**Independent safeguards during the walk** (each one commands zeros and ends the test as `SAFETY_ABORT`):

| Failure | Detected by |
| --- | --- |
| Stale or missing LiDAR | Terrain status older than `--max-sensor-age`; rate below `--min-lidar-hz`; hazard cloud older than 0.5 s; lock input `/lite3_terrain/stop` stale |
| Invalid IMU or joint telemetry | Every tick: IMU orientation valid; joint names, count and ranges valid. Telemetry older than 0.25 s |
| Lost obstacle tracking | A tracked obstacle is missing for `--lost-track-scans` scans while it must still be visible |
| Unexpected obstacle measurement | The distance disagrees with the robot's own travel by more than `--track-tolerance`, either between two scans or in total, allowing up to 25 % odometry under-read. Typical causes: a moving object, or something stepping into the path |
| Proximity-lock failure | The obstacle is inside the 0.30 m backup stop but the lock input is false |
| Protective stop failure | The obstacle is inside the boundary but forward commands still reach the robot `--response-limit` s later. The test itself then stops the robot |
| Unexpected motion | Lateral drift > 0.15 m, heading change > 0.25 rad, backwards > 0.05 m, odometry speed above 2× the commanded speed for 0.5 s |
| Excessive command speed | Any robot-side command faster than the authorised `--approach-speed` |
| Deadman timeout | A command-scheduler gap of more than 3 cycles aborts. A dead-man that lapses anyway is reported as `UNEXPLAINED_STOP` |
| Travel, time, area | `--max-travel`: leg odometry ÷ 0.8, **and** the SLAM map pose. `--max-duration`. Odometry geofence at travel + 0.2 m. Hard floor: an obstacle 0.10 m from the nose aborts. Odometry budget to the last good obstacle measurement |

**M4 parameters**: command line, or a YAML profile (`--profile`; the command line wins). See
`operator/run_physical.sh --help`. Values outside the hard limits are refused, not clamped.
Raising the travel or the duration never changes the speed, the boundary or any safeguard.

| Option | Default | Allowed |
| --- | --- | --- |
| `--approach-speed` | 0.05 m/s | 0.02–0.05 |
| `--max-travel` | 1.5 m | 0.2–1.8 |
| `--max-duration` | travel ÷ speed + 10 s | 5–60 s |
| `--min-clearance` (`--final-clearance`) | navigation `--clearance` | 0.10–1.0 m |
| `--trigger-distance` | the running boundary | must **equal** it; change it with `start_navigation.sh --clearance` |
| `--initial-clear` | boundary + 0.15 m | boundary + 0.05 – detection range |
| `--detection-range` | 2.0 m | 0.5–2.5 m (terrain corridor) |
| `--max-sensor-age` | 0.30 s | 0.10–0.50 s |
| `--min-lidar-hz` | 8 Hz | 8–20 Hz |
| `--detect-scans` | 3 | 2–10 |
| `--track-tolerance` | 0.15 m | 0.05–0.25 m |
| `--lost-track-scans` | 3 | 1–5 |
| `--suppression-proof` | 1.0 s | 0.5–1.5 s |
| `--response-limit` | 0.30 s | 0.10–0.50 s |

Profiles in `operator/profiles/`:
- `m4_default.yaml`: the defaults, written out.
- `m4_first_session.yaml`: 0.03 m/s, 1.0 m, stricter freshness.
- `m4_strict_sensing.yaml`: more confirming scans, tighter tracking.

**Dry runs** (no `--physical`; nothing is published):
- `--phases M4` reports READY or the exact blockers.
- `--phases M4 --shadow [--shadow-seconds 60]` runs the same detection and stop logic on live
  sensing while the robot stands still. Slide the chair slowly toward the robot along its
  heading. The report shows where the chair was confirmed, when the stop would trigger, whether
  the backup lock engaged, the sensor health and the physical M4 blockers.

**Setup**:
- A straight, flat lane: `--max-travel` + 0.5 m long and 0.6 m wide.
- Nobody in the lane.
- Nothing closer than `--initial-clear` at the start.
- The chair anywhere in the lane, up to `--detection-range`.
- For the first sessions, put it 0.8–1.2 m ahead. It is then visible from the start, so lost
  tracking and the odometry budget to the chair are armed from the first step.

**Residual risk**: one fault defeats every sensing layer at once. If the LiDAR goes blind while
still reporting healthy, fresh "nothing ahead" data, and the obstacle was **not** visible at the
start, then only the travel and time limits and your e-stop remain.

M3 needs only M0: the robot is expected not to move, and the first forward command that
reaches `motion_sender` aborts it. M4 deliberately drives toward an obstacle, so it still needs
M2 (verified stopping) as well as M3.

The session stops at the first test that does not PASS. Later tests depend on
earlier ones, and the order is enforced: **a test is refused until its
prerequisites have passed on the same navigation run** (restarting the stack
resets this):

| Test | Moves | Needs | Setup | PASS (from robot telemetry) |
| --- | --- | --- | --- | --- |
| **M0** command path | zeros only | none | Robot at rest, nobody within 1 m | Nav2, terrain and LiDAR ready; twist_mux is the only `/cmd_vel` source; zeros reach `motion_sender` within 0.5 s; nothing passes while idle |
| **M1** basic motion | ~0.1 m forward, turns in place | M0 | 1 m clear all round | Each move reaches `motion_sender` through the mux; leg odometry displacement and heading in range; rest after every move |
| **M2** stop paths | ~0.1 m forward | M1 | 1 m clear all round | After the dead-man is released (while the test **keeps** commanding): commands at `motion_sender` stop within 0.8 s; the robot **stops moving** (no translation, no foot lifts: speed ≤ 0.04 m/s, leg-extension range ≤ 0.02 m, position held to 0.02 m over 1.5 s) within **3 s**; it is **fully settled** (every rest signal quiet for 1.5 s) within 5 s, turning ≤ 0.05 rad and shifting ≤ 0.02 m while settling |
| **DET** object detection | nothing | none | Object straight ahead; `--object-distance` = its distance from the nose (optional); nobody within 1 m | Seen ahead in ≥ 95% of LiDAR scans over 5 s, no status gap > 0.3 s, distance spread ≤ 0.05 m. Read-only in dry mode: run it with the chair/box in place before M3/M4 |
| **M3** proximity block | must not move | M0 | Box ≥ 20 cm tall, front face (stop − 0.10) m from the nose (default 0.20 m) | First, 3 s without commands: the object is seen steadily **and** holds the stop as an object *ahead* (not "within 0.45 m"), lock input never false. Then the lock is re-confirmed before **every** forward command (otherwise the test withholds it and aborts); no forward command reaches `motion_sender`; moves < 3 cm |
| **M4** sensor-driven proximity stop | walks straight ahead until something stops it (≤ `--max-travel`, ≤ `--max-duration`) | M2, M3 | Straight lane `--max-travel` + 0.5 m long, 0.6 m wide; nothing within `--initial-clear` at the start; chair/box **anywhere** in the lane up to `--detection-range` (no tape measure needed). Navigation started with `--clearance` ≥ 0.19 (boundary ≥ 0.35 m) | Outcome `STOPPED_FOR_OBSTACLE`: moved; obstacle detected and tracked; the collision_monitor stopped the robot **for it** while forward commands were still published and zeroed at the robot; suppressed within the response limit; stopped moving within 3 s; minimum and final clearance ≥ `--min-clearance`. Timer, travel limit, backup lock, operator and safeguard stops are separate outcomes, never PASS. Details: §3.1 |
| **M5** straight path | Nav2, goal ahead | M2 | Lane: goal + 1 m, 0.6 m each side | Nav2 SUCCEEDED; final map pose within tolerance; leg odometry agrees with the map; footprint clearance ≥ `--clearance`; speed ≤ `--max-speed`; no reverse |
| **M6** turn / junction | Nav2, goal around a corner | M5 | L-shaped clear area (default 1.0 m ahead, 0.8 m left, +90°) | As M5, including the heading |
| **M7** avoidance | Nav2 past a box | M5 | Lane ≥ 1.2 m each side; box on the straight line halfway | As M5, **and** the path deviates ≥ 0.15 m from the straight line |
| **M8** blocked goal | Nav2 toward a barrier | M5 | Barrier fully across about 1.2 m ahead | Goal **not** reached; no collision; Nav2 aborts or times out; at rest; recoveries recorded. Then remove the barrier and repeat M5 |
| **M9** course | Nav2 through waypoints | M6, M7 | Course laid along `--waypoints` (forward,left[,yaw°] from the start pose) | Every waypoint reached in order (M5 criteria each); course geofence held |
| STAND / SIT | posture only | M0 / none | 1 m clear | Posture verified by joint geometry, IMU and rest |

The test catalogue with your current parameters: `operator/run_physical.sh --list-phases`.

**M10 autonomous roaming** is not part of this sequence and needs none of these tests: see
[README.md §0.1](README.md) and `operator/roam.sh help`.

## 4. Parameters (all optional; every one is checked against the hard limits)

| Flag | Default | Allowed | Used by |
| --- | --- | --- | --- |
| `--max-speed` | 0.05 m/s | 0.02–0.5 (above 0.10: §1.1) | M1–M3 moves, Nav2 speed limit (M5–M9), robot-side abort cap |
| `--max-yaw-rate` | 0.15 rad/s | 0.05–0.20 | M1 turns |
| `--stop-distance` | 0.30 m from the nose | 0.30–1.50 | Backup proximity stop for the test (restored afterwards). M3/M4 always use 0.30 m |
| M4 options | see §3.1 | hard limits in §3.1 | M4 (`--profile`, `--shadow`, `--max-travel`, ...) |
| `--clearance` | 0.10 m | 0–1.0 | Minimum obstacle-to-footprint clearance for PASS |
| `--move-seconds` | 2.0 s | 1–3 | M1–M3 move length |
| `--goal-distance` | M5/M6 1.0 m, M7/M8 2.0 m | 0.3–6 | Goal ahead |
| `--goal-left`, `--goal-yaw-deg` | 0.8 m, 90° | ±6 m, ±180° | M6 |
| `--xy-tolerance`, `--yaw-tolerance-deg` | 0.25 m, 30° | 0.05–1.0 m, 5–180° | PASS tolerance at the goal (Nav2's own goal checker is unchanged) |
| `--timeout` | from distance and speed | 10–240 s | Per Nav2 goal (M8 default 90 s) |
| `--waypoints`, `--course-radius` | `1,0;2,0`, auto | ≤ 6 m each, radius ≤ 10 m | M9 |
| `--sim`, `--manual` | off | | Rehearsal / pause before each test (dry or sim) |

Examples: `operator/run_physical.sh --physical --phases M3 --stop-distance 0.40`;
`operator/run_physical.sh --physical --phases M4 --max-travel 1.2 --min-clearance 0.20`.

Test 4 (`run_test4.sh`) stays perception-only. `run_test4.sh ... --physical`
explains the physical counterpart of each phase (P3 → M3, M4; P2 → M7; P1 → M1)
and exits without doing anything. For the dry test, `run_test4.sh --stop-distance
0.40` also works.

## 5. Stopping

| Situation | Do this |
| --- | --- |
| Robot does anything unexpected | **Handheld remote / e-stop first.** Then Ctrl+C. |
| Stop the current test | Ctrl+C in its terminal, or `operator/abort_physical.sh` from another one. The test cancels the Nav2 goal, commands zeros, verifies rest and writes the report. |
| End of session | `operator/run_physical.sh --physical --phases SIT`, then `operator/stop_navigation.sh` (it also stops a running test first) |
| Verdict INTERVENTION REQUIRED | The robot's state could not be verified. Check the robot physically. Use the remote if needed. Read the report before running anything else. |

## 6. Results

- **Location:** each test writes `report.md` and `report.json`, plus the exact
  `sequence.yaml` and the `authorization.yaml` generated from your confirmation.
  They are under `log/operator/run-<start>/physical/<session>/<time>-<test>/`.
  Passed tests are listed in `log/operator/run-<start>/physical/gates.json`.
- **Verdicts:**
  - PASSED: every criterion was met, measured by robot telemetry.
  - FAILED_SAFE: a criterion was missed, and rest was verified afterwards.
  - INTERVENTION_REQUIRED: the robot's state is uncertain.
- **Evidence:** a published command or Nav2 "SUCCEEDED" is never counted as a
  pass by itself.

## 7. Known limits and blockers (state on 2026-11-17)

- **M4 redesigned (2026-11-17), not yet run physically.** It no longer needs the obstacle at a
  start distance (12 of 14 M4 attempts on 2026-11-16 were refused only for that). Validated in
  software:
  - 67 dedicated tests with fault injection;
  - the real collision_monitor: first zero 1 ms after the crossing scan, none early, none after;
  - live read-only READY and shadow checks against the robot.

  It **refuses** on a stack started with the default `--clearance 0.15` (boundary 0.31 m, too
  close to the 0.30 m backup lock). Restart navigation with `--clearance 0.20`.
- **M5 overspeed (2026-11-16) root-caused and fixed in software.**
  - Cause: Humble MPPI resets its optimizer when a goal arrives and drops the speed limit sent
    before it. Nav2 commanded 0.064–0.098 m/s against 0.05.
  - Fix: every Nav2 goal first sets and reads back `velocity_smoother.max_velocity`, which clamps
    after MPPI. The speed limit is re-sent after the goal.
  - Verified on the real velocity_smoother (`tools/check_velocity_cap.py`): 0.10 in → 0.05 out.

  Re-run M5 at `--max-speed 0.05` to confirm on the robot.
- **M2 dead-man failure (2026-11-16)** was resolved by the two-stage criterion (see
  `src/lite3_movement_tests/docs/trial_2026-11-16_m2_deadman_rootcause.md`). M2 passed 3 times
  since, including 2026-11-17 10:45. **M3 command leak** was resolved by re-confirming the lock
  before each command. M3 passed twice since.

- **Not yet run on the robot.** All of this was validated in software:
  - Unit tests with fault injection.
  - The real twist_mux binary, in an isolated domain.
  - The real terrain monitor, in an isolated domain.
  - A live read-only check against the robot.

  The first physical session starts at M0.
- **Stairs are not supported.** No stair gait is wired in, and the terrain
  monitor treats stairs as obstacles or drops. M9 courses must route around them.
- **Low objects** (Test 4 P4): detected, but stepping over them is not validated.
  There is no physical test for it.
- **Navigation mode** can't be read from telemetry. M1 checks it by behaviour.
  **The motion firmware's command timeout is unverified.** The measured coast
  after silence is about 1.6 s.
- **No reversing:** the LiDAR doesn't see behind the robot. Reverse commands
  abort the test, and recovery behaviours exclude BackUp.
- **Each `start_navigation.sh` builds a fresh SLAM map.** Goals are relative to
  where the robot is when the test starts.
- **The domain bridge creates its `/cmd_vel` relay only once a publisher
  exists.** On a dry stack, the read-only check therefore reports "bridge relay
  not yet created"; M0 verifies the relay on the armed stack.
