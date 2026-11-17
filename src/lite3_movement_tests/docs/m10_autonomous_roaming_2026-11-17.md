# M10 autonomous roaming (2026-11-17)

Status: implemented and validated in software. **Not yet run on the robot.**

## Design

M10 is independent of M0–M9:
- It is not in the `physical_suite` catalogue.
- It reads no gates and no earlier reports. A test asserts this.

Its own readiness runs `SequenceRunner` steps:
1. `readiness`
2. `nav_readiness`, including the collision-boundary re-check against live sensing delay
3. `telemetry_contract`
4. `initial_rest`
5. `zero_delivery`: zeros reach `motion_sender`, nothing passes while idle
6. `stand`
7. `verify_standing`

The same runner instance then supplies the per-tick safety check (`_check`). That check covers
telemetry, tilt, the remote, command-path exclusivity, robot-side speed against the
authorisation, Nav2 activity outside roaming, and the map-pose geofence. The dead-man is fed only
when every check passes.

Components reused unchanged:
- SLAM (slam_toolbox via the university launch)
- Nav2 (planner, MPPI, costmaps with the terrain hazards)
- velocity_smoother cap (the M5 fix)
- twist_mux locks
- collision_monitor
- `lite3_locomotion.frontiers.select_frontiers`

Code:

| File | Contents |
| --- | --- |
| `roam.py` | Config and hard limits, control channel, explorer, watchdog logic, session state machine, CLI |
| `roam_ros.py` | ROS adapters |
| `roam_watchdog.py` | Independent process |
| `roam_sim.py` | Stand-in |
| `operator/roam.sh` | Operator script |

Changes to shared code, all backward compatible:
- `SequenceRunner` honours an optional `session_limit_s`.
- `RosRobot.node_name` is a class attribute.
- `Telemetry` has two new streams.
- The simulated smoother also caps yaw (M5 sets it to 0.20, so M5 is unchanged).
- `run_physical.sh` refuses while a physical roam runs.

### Speed modes vs gait modes

The vendor ROS bridge (`Jetson2Motion.cpp`) maps `/cmd_vel` to velocity commands 320/325/321 and
forwards `/simple_cmd` codes. The only documented `/simple_cmd` codes here are:
- posture toggle `0x21010202`;
- `0x21010C02` / `0x21010C03` (Manual / Navigation).

No gait-switch code exists in the vendor source, the university workspace or the live graph.
`robot_gait_state` is dropped by the deployed `motion_receiver`, and no ROS topic carries it
(live listing of domains 8 and 64). A gait command could therefore be neither sent from a verified
source nor confirmed. M10 never sends one.

`slow` / `normal` / `fast` are velocity envelopes in the robot's standing gait. They are enforced:
- by the `velocity_smoother.max_velocity` cap, linear and yaw, read back;
- by the MPPI speed limit, re-sent after each goal and every second;
- at the robot side by the runner, per mode, and by the authorisation, per session limit.

### Point-cloud enhancements

- **Exploration**: a goal is vetoed if the Nav2 global costmap has a cell ≥ 70 within the goal
  clearance. That costmap already contains the 3D hazards (`terrain_monitor` from `/livox/points`).
  This avoids targets at tables, low objects and drops that the 2D SLAM map misses.
- **Detection health**: raw `/livox/points` rate and serialized size, and `/cloud_registered_body`
  freshness, are checked in the runner and the watchdog. This addresses the common-mode residual
  risk found in M4: a blind LiDAR while the derived status still looks healthy.
- The clouds, map and costmap are subscribed on a **separate node, context and executor**, and the
  clouds **raw (never deserialized)**. A first live dry run showed the 0.5 MB clouds on the
  telemetry executor delayed watchdog readiness from about 10 s to about 40 s, which could also
  starve the safety-relevant joint and IMU callbacks.

### Independent watchdog

The watchdog is a separate process with its own node (`lite3_roam_watchdog`). It has no publishers
while the runner lives, so the runner's exclusive-wiring check stays valid.

Its rules use looser limits and independent sources where possible:
- telemetry older than 0.5 s;
- tilt above 0.40 rad;
- raw LiDAR stale or empty;
- robot-side speed above the limit + 0.02, or reverse;
- leg-odometry speed above max(2× limit, limit + 0.05);
- leg-odometry distance above geofence + 0.75 m;
- runner heartbeat older than 2 s (15 s during readiness).

When it trips:
1. SIGINT to the runner, which then stops the robot itself.
2. SIGKILL after 1.5 s if the runner has not stopped. The dead-man then locks within 0.5 s.
3. Once the runner is gone, zeros on the priority-255 stop input and `hold=true` for 3 s.

The runner stops roaming if the watchdog's heartbeat is lost.

## Findings during implementation

1. The simulated Nav2 ignored the yaw cap. The runner correctly stopped on a 0.20 rad/s command in
   slow mode (0.15). The simulator was fixed; the real smoother caps all three axes.
2. A runaway gait (actual speed far above the command, inside the geofence) was not caught at
   first. An actual-speed check was added to the runner and the watchdog.
3. An inverted grace condition loosened the actual-speed limit outside the grace window. It was
   fixed and covered by a dedicated test.
4. Large point-cloud callbacks starved telemetry (see above). Fixed by isolation and raw
   subscriptions.

## Validation (no actuation)

- `test/test_m10.py`: 35 tests covering independence, readiness content, nominal roaming with
  pause/resume/speed/stop, no motion while paused, per-mode robot-side speeds, refusal above the
  session limit, a dry run that publishes nothing, the point-cloud goal veto, 8 safety-stop
  causes, runaway, geofence, config limits, the control channel and every watchdog rule.
- `operator/roam.sh start --sim`: 200–300 s rehearsals with commands. Outcome STOPPED, rest
  verified.
- The watchdog process run standalone against the live stack (read-only): ready in about 10 s.

## Residual limits

- Not run on the robot.
- The Lite3 can be stopped only by commands. Navigation mode disables handheld walking, so the
  manufacturer e-stop / remote procedure is the emergency stop.
- Frontier exploration uses the 2D SLAM map. Areas visible only in 3D are vetoed as targets but
  not explored as frontiers.
- Stairs are unsupported. The operator must keep them out of the geofence.
