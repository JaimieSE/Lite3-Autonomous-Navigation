# lite3_movement_tests

This package runs automated, bounded Lite3 movement-test sequences. It uses the
**existing** university/vendor bridge and its telemetry. Every step passes only when
measured telemetry meets that step's criteria, never because a command was
published. A physical session is authorized once, in one file. After that the
runner works through the steps without per-step prompts. It stops, and asks for a
person, only when something fails or the robot's state is uncertain.

This package is deliberately separate from `lite3_locomotion`. That package's
metadata tests forbid actuating or `transfer_interfaces` code in the autonomy
install, and that boundary is preserved. No driver, bridge, telemetry publisher,
Nav2 instance or UDP sender is added.

## What is reused

| Existing component | Used for |
| --- | --- |
| `motion_receiver` (vendor `Jetson2Motion.cpp`, system domain 64) relayed by `lite3_domain_bridge_8` | `/joint_states` (12 named positions), `/leg_odom2` (pose + body twist), `/imu/data`, all at 200 Hz |
| vendor `sensor_checker` | `/sensor_status/{imu,joint,odom2}_isalive` readiness |
| `/handle_state` (handheld remote axes) | operator-takeover detection: any stick input aborts motion |
| `lite3_domain_bridge_8` → `lite3_domain_bridge_64` → `motion_sender` | velocity commands on group-domain `/cmd_vel` (the path used in both supervised trials) |
| `motion_sender` `/simple_cmd` in domain 64 | one volatile `0x21010202` stand/sit toggle (the code that the university `drain_patrol` and both trials use) |
| Lite3 URDF (`~/lite3_visualization/.../Lite3.urdf`) | leg link lengths for posture geometry |

Nothing in `~/ros2_ws` (university) or the vendor tree was modified.

## Posture feedback: what exists and what is used

- **No posture, mode or gait acknowledgment is available over ROS.** The firmware
  state packet contains `robot_basic_state`, `robot_gait_state` and
  `robot_motion_state` (vendor `protocol.hpp`), but the deployed `motion_receiver`
  drops them. The live graph in domains 8 and 64 has no such topic, and neither the
  university workspace nor the vendor source publishes one. Exposing them would
  mean changing the vendor bridge, which is out of scope and unauthorized.
- **The `/leg_odom2` pose is not a posture signal.** While the robot was lying, it
  stayed frozen at z = 0.3227 m (a live 10 s sample showed zero change).
- **The criterion used here is joint geometry.** Each leg's hip-to-foot distance
  is computed from |knee| and the URDF link lengths (0.200 m and 0.21012 m). This
  uses only the magnitude, so the bridge's sign convention does not matter.
  - **Lying:** all four legs ≤ 0.14 m. The live robot measured 0.080–0.082 m.
  - **Standing:** all four legs ≥ 0.22 m and within 0.06 m of each other.
  - **Both postures also require:** IMU roll and pitch ≤ 0.20 rad, and every joint
    varying ≤ 0.05 rad over the hold window (settled).
  - **Validation status:** the lying band has been confirmed on the live robot. The
    standing band is a geometric prediction (a nominal stance gives about 0.28 m).
    It counts as confirmed only after a recorded physical stand transition, and
    every report states this.
- **A stand or sit toggle is sent only from a verified opposite posture, at rest.**
  It is sent at most once. It passes only if telemetry first leaves the source
  posture and then holds the target posture. On any posture failure, posture
  commands are latched off for the rest of the session and an operator is
  requested. The toggle is never retried, because a second toggle could undo the
  first.

## Motion verification

**Rest** requires all of the following over the hold window. The thresholds sit
well above the measured rest noise (odometry ≤ 0.015 m/s, joints < 0.001 rad):

- joints varying ≤ 0.05 rad
- odometry speed ≤ 0.04 m/s and yaw rate ≤ 0.05 rad/s
- odometry position change ≤ 0.02 m
- IMU angular rate ≤ 0.15 rad/s

A **move** step runs in this order:

1. **Precondition:** standing is verified and the robot is at rest.
2. **Warmup:** 0.5 s of zero commands.
3. **Constant command:** published at 20 Hz.
4. **Zero burst:** 2 s, issued even after an abort.
5. **Measurement:** displacement in the start-heading frame.

The step passes only if the measured `forward_m`, `lateral_m` and `yaw_change_rad`
fall within the sequence's configured ranges.

The report also records peak odometry speed, joint activity and how many commands
reached domain 64. When joints move without translation, the failure is diagnosed
as "stepping without translation". That is what the 2026-11-13 trials observed.

**Continuous safety checks** run during every actuating step. Any one of these
aborts motion, sends zeros and ends with `INTERVENTION_REQUIRED`:

- stale joints, odometry or IMU (more than 0.25 s old)
- tilt above 0.35 rad, or an invalid IMU quaternion
- any handheld remote stick input
- a geofence breach (0.5 m from the start of each move)
- an unexpected publisher or subscriber on `/cmd_vel`, `/cmd_vel_corrected`,
  `/simple_cmd` or `/complex_cmd` in either domain
- nonzero commands not appearing in domain 64 within 0.5 s
- a scheduler gap, the session time limit, or Ctrl+C / SIGTERM

**Limits in code** cannot be raised by any configuration or authorization:

| Limit | Value |
| --- | --- |
| Speed | 0.10 m/s |
| Yaw rate | 0.30 rad/s |
| Move duration | 3 s |
| Posture timeout | 20 s |
| Session | 600 s |
| Authorization window | 4 h |

Every `velocity_move` must be followed by a `stop_verify` step.

**Finalize steps** (for example, lying down at the end) run only when this session
sent commands and the state is still verified. If nothing was sent, the robot is
left as it was found.

## Default sequence (`config/basic_walk.yaml`)

1. Readiness
2. Telemetry contract
3. Initial rest
4. Stand: a toggle only from verified lying; skipped if already standing
5. Verify standing
6. Forward move: 0.03 m/s for 2 s, expecting 0.03–0.15 m forward, |lateral| ≤ 0.08 m, |yaw| ≤ 0.2 rad
7. Stop verification
8. Finalize: lie down

To add tests, add steps. For example, a turn in place:

```yaml
type: velocity_move
wz: 0.15
duration_s: 2
expect: {yaw_change_rad: [0.15, 0.45], forward_m: [-0.05, 0.05]}
```

Follow it with a `stop_verify` step. Any change to the file changes its sha256,
which invalidates existing authorizations.

## Commands

```bash
cd ~/lite3_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select lite3_movement_tests

# Unit tests (no ROS, no robot)
python3 -m pytest -q -p no:cacheprovider src/lite3_movement_tests/test
# DDS end-to-end against the simulator (isolated random domain 40-89, loopback only)
LITE3_MT_RUN_DDS=1 python3 -m pytest -q -p no:cacheprovider src/lite3_movement_tests/test/test_dds_sim.py

ros2 run lite3_movement_tests movement_test plan
ros2 run lite3_movement_tests movement_test dry-run --sim [--fault march_in_place|ignore_posture|remote_takeover|tilt|telemetry_dropout|no_stop|slow_stand|runaway] [--start standing]
ros2 run lite3_movement_tests movement_test dry-run --live-readonly   # real telemetry, NO publishers created
```

### Authorized bounded physical session

1. Run `movement_test dry-run --live-readonly`. It must report `READONLY_CHECKS_PASSED`
   and show the stand precondition verified.
2. Create the session authorization:
   `ros2 run lite3_movement_tests movement_test authorize-template --output ~/lite3_ws/log/auth_today.yaml`
3. At the robot, check each item physically. Then set its confirmation to `true`
   (literal `true`, not "yes"):
   - area clear for 1 m
   - emergency stop in hand and tested
   - exclusive control of the shared dog
   - Navigation/Auto mode selected
   - operator supervising throughout
   - accept no LiDAR collision gate
   - accept an unverified firmware command timeout
4. Lower the template limits if you want; it starts at the sequence's own values.
   `valid_until` is at most 4 h away.
5. Run `ros2 run lite3_movement_tests movement_test physical --authorization ~/lite3_ws/log/auth_today.yaml`.
   - It refuses (exit 3, nothing published) if the authorization is incomplete or
     expired, the sequence hash differs, the session is out of scope, or the
     environment is localhost-only.
   - Otherwise it runs after a 3 s countdown with no further prompts. Ctrl+C aborts
     with zeros.

Exit codes:

| Code | Meaning |
| --- | --- |
| 0 | Passed |
| 1 | Failed, robot verified safe |
| 2 | Intervention required |
| 3 | Refused before any command |

Reports go to `~/lite3_ws/log/movement_tests/<time>-<mode>/` (override with
`--report-dir` or `LITE3_MOVEMENT_REPORTS`):

- `report.md` is the summary, failures and final state.
- `report.json` holds every measurement, the events, and a 20 Hz telemetry trace
  for each actuating step.

## Evidence classes

Every report states its class, and they are never merged:

| Class | What it shows |
| --- | --- |
| `software simulation (no robot)` | Proves runner logic over DDS only. The simulator is a stand-in, not a physics model. |
| `live robot telemetry, no commands published` | Real readiness, contract, rest and posture classification. |
| `physical robot telemetry` | The only class that can support claims of physical movement, and only through its measured odometry and joint data. Leg odometry is the vendor estimator, not ground truth. |

## Known limits

- The vendor `motion_sender` has no command timeout. Firmware behavior if this
  process dies mid-command is unverified. The runner sends zeros on every exit
  path it controls, but SIGKILL, power loss or a network loss cannot be covered.
- There is no LiDAR collision gate (`/scan` is not running in the group domain).
  The clear area is an operator confirmation.
- Navigation/Auto mode is not observable. It is an operator confirmation.
- The rclpy receive rate is about 110 Hz of the 200 Hz published. Freshness (0.25 s)
  is the safety check, not the rate.
