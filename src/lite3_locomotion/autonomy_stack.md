# Nav2 and guarded movement adapter

The separate autonomy stack is implemented and starts **dry-run and disarmed**.
No driver, SDK bridge, robot service, posture command, or robot mode transition
is started by its launch file. The original fixed-topic test controller and
marker-only exploration preview retain their non-actuating behavior.

## Data flow

```text
Existing /map + /scan + /odom + /tf + /tf_static
  -> frontier coordinator -> /lite3_autonomy/navigate_to_pose (Nav2 action)
  -> Navfn planner + Regulated Pure Pursuit controller
  -> /lite3_autonomy/nav_cmd_vel (Twist)
  -> guarded adapter
       default dry-run -> /lite3_locomotion/guarded_test_cmd_vel
       explicit robot  -> /lite3_robot_link/cmd_vel
                          -> separately deployed vendor motion_sender -> motion host
```

[exploration_coordinator.py](../lite3_locomotion/exploration_coordinator.py)
chooses reachable goals on known free cells near unknown space. It uses the
existing SLAM map, which is derived from LiDAR; Nav2 additionally uses `/scan`
for obstacle costs. Unknown cells are excluded from global planning. No
preplanned route is required. No automatic spin, backup, recovery behavior,
gait selection, joint control, drop-off detector, or terrain classifier is
included. This is an initial flat-floor exploration stack.

[guarded_adapter.py](../lite3_locomotion/guarded_adapter.py) is the new movement
command gate. [movement_guard.py](../lite3_locomotion/movement_guard.py) implements
its deterministic policy. The wire protocol reference is the pinned
[vendor movement bridge](../vendor/Lite3_ROS/src/transfer/src/Jetson2Motion.cpp),
`MotionSender::CmdVelCallback`. The separate [motion sender](robot_output.md)
implements the opt-in transport. The adapter itself opens no UDP connection.

## Defaults and bounds

| Check | Implemented behavior |
| --- | --- |
| Startup | Disarmed; dry-run publishes zero; robot mode publishes nothing until explicitly armed |
| Commands | Finite Twist fields; reject vertical/roll/pitch; clamp x to ±0.10 m/s and yaw to ±0.20 rad/s; lateral output zero |
| Fresh commands | 0.40-second monotonic receipt timeout; first command required within 2 seconds of arming |
| Stops | Invalid command, timeout, unhealthy sensors/TF/link, or software stop latches disarmed; fresh commands cannot rearm |
| Session | 60 seconds by default, maximum 300; repeating arm while armed does not extend it |
| Area | Default 3 m radius from arm/start position; goals remain at least 0.4 m inside mission radius |
| Scan | `base_link` frame, roughly 360° coverage, receipt and header age ≤0.5 s, at least 80% usable beams and 3 finite returns; stop for a valid return closer than 0.40 m |
| TF | `map -> base_link`, finite x/y, stamp age ≤0.5 s; future stamps beyond 0.2 s rejected |
| Map | Valid occupancy grid received within 30 seconds; bounded search sizes inherited from frontier preview |
| Nav2 | All three lifecycle nodes must report active; state responses must remain fresh within 2 seconds |
| Mission | Maximum 20 goals, 30 seconds per goal; stop after 3 consecutive failures or no eligible frontier |
| Robot output graph | Exactly one expected motion_sender subscriber and one controller_server input publisher; no additional output publishers |

The numeric velocity limits, circular footprint radius (0.36 m), inflation,
clearances and timing values are **placeholder settings**, not measured Lite3
capabilities or a verified operating envelope. Units follow ROS conventions;
actual firmware scaling and response still require deployment verification.
Receipt-time Twist expiry cannot detect old messages replayed continuously by
a faulty upstream source. ROS graph names are checks against wiring mistakes,
not authentication or proof of motion-host identity.

## Build and isolated verification

Verified in this Humble environment on 2026-11-11:

| Validation | Result |
| --- | --- |
| Isolated package build | Passed |
| Ordinary unit suite | 241 passed; 17 opt-in integration cases skipped |
| New isolated Nav2 integration suite | 6 passed, including start/stop CLI and cancellation |
| Original isolated ROS test-controller suite | 11 passed, including remapping rejection |
| Launched integration processes | All exited; no robot bridge launched |
| Physical motion or mode switching | Not performed |

All generated build, test and runtime files can remain inside this package:

```bash
cd /home/user/lite3_ws/src/lite3_locomotion
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$PWD/.colcon/log" build \
  --base-paths "$PWD" --packages-select lite3_locomotion \
  --build-base "$PWD/.colcon/build" --install-base "$PWD/.colcon/install"
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test
PYTHONDONTWRITEBYTECODE=1 python3 tools/run_autonomy_integration.py
```

The last command starts actual Nav2 nodes, adapter and coordinator against a
small synthetic map and synthetic scan/TF/odometry. It forces a random domain
40–89, local-only Fast DDS using UDP on `127.0.0.1`, no SHM or discovery server,
and **dry-run output**. No robot bridge is launched. It verifies nonzero Nav2
commands reach only the dry-run output, plus startup, limiting, stale-command
latching, obstacle/header freshness stopping, and mission cancellation. Results
and process logs are in `.colcon/autonomy_integration/`. Normal unit tests skip
ROS integration cases. This is not a physics simulation or physical motion test.

## Dry-run with the existing live map

Run on the host containing the package and existing Nav2 dependencies; the
RViz VM does not automatically contain this installation. Source the isolated
installation in every terminal. Existing domain-8 SLAM, scans and TF must already
be running. Do not launch another SLAM or sensor driver.

Terminal 1:

```bash
source /opt/ros/humble/setup.bash
source /home/user/lite3_ws/src/lite3_locomotion/.colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
export ROS_LOG_DIR=/home/user/lite3_ws/src/lite3_locomotion/.colcon/live_dry_run_logs
ros2 launch lite3_locomotion autonomy.launch.py output_mode:=dry-run
```

Wait for the lifecycle manager to report all managed nodes active. Terminal 2,
with the same setup/domain/log environment:

```bash
ros2 run lite3_locomotion autonomy_control start
ros2 topic echo /lite3_locomotion/guard_status
```

This explicitly arms the **dry-run** adapter and starts a bounded mission.
The robot will not move. The static robot position may cause Nav2's progress
checker to abort after 15 seconds; that is expected without simulated motion.
Inspect the path/costmaps in RViz under `/lite3_autonomy`, and inspect
`/lite3_locomotion/guarded_test_cmd_vel`. The coordinator sends goals directly
to its namespaced action server; RViz's default goal tool is not used.

To stop, from a sourced terminal in domain 8:

```bash
ros2 run lite3_locomotion autonomy_control stop
```

The tool requests the adapter stop first, then mission cancellation, and reports
service responses. Stop remains latched until a new explicit start. After the
acknowledgment, Ctrl+C in Terminal 1 ends the launched stack. A component exit
also shuts down the rest of the launch. No existing services are restarted.

## Robot-output integration status

An opt-in, separately armed [UDP motion sender](robot_output.md) is now provided.
It adds lower commissioning speed limits, a separate-process command watchdog,
and an enabled-sender heartbeat required by the adapter in robot mode. Physical
deployment, firmware and motion-host stopping remain unverified. The autonomy
launch still does not start the sender or change robot modes.

`output_mode:=robot` is a separate, explicit output selection. It still starts
disarmed, but arming and starting it can produce physical velocity commands
when a compatible bridge is present. It is **not** used by the tests or by the
dry-run instructions above. The legacy test controller cannot be converted
into this adapter by remapping or parameters.

The adapter expects a separately deployed node named `motion_sender` in
`/lite3_robot_link` subscribing to `/lite3_robot_link/cmd_vel`. The supplied
launch does not start that bridge, contact the motion host, stand the robot,
enter Navigation mode, or restore Manual mode. Manual-takeover integration is
omitted as requested. Its stop service requests zero velocity; it does not
return the vendor controller to Manual mode.

[The earlier bridge review](movement_test_review.md) remains the source of
verified protocol/deployment findings: command-free startup was checked, while
firmware compatibility, actual velocity units/signs, active motion-host
configuration and independent command-expiry/stop behavior remain unverified.
The upstream address is a source default, not an established deployment value.
The separate receiver-shutdown fix does not add a motion watchdog.

Software zeros and bounded shutdown publication are best effort. If this
process, its computer or the link fails, the adapter cannot send a stop. The
vendor bridge source has no independent command-expiry timer. Consequently,
this implementation alone does not establish crash-safe physical stopping.
No physical movement or mode changes have been performed to validate it.

The next development step is to run the isolated synthetic test, then inspect
dry-run planning against the existing live map. Physical commissioning needs
the remaining movement-interface evidence; it is separate from creating and
validating this stack.
