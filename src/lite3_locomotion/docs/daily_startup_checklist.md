# Lite3 daily startup: return to the verified milestone

Prepared 2026-11-12 for group 8, robot 4, ROS 2 Humble.

## 1. Know what this procedure does

Current demonstrated milestone:

- The university System Computer runs `transfer/jetson2motion`.
- A single stand/sit toggle from group domain 8 was observed in System domain
  64, and the operator confirmed that the robot stood.
- LiDAR scans have been available to SLAM, Nav2 costmaps and the custom guard.
- Custom navigation/exploration has been exercised in dry-run, not commissioned
  for physical autonomous walking.

This guide restores those components. It does **not** enable autonomous walking.
The guarded university velocity-bridge integration, walking/stop verification,
measured footprint and stopping margins, and communication-loss behavior remain
unfinished. Do not turn dry-run output into a physical command by remapping it.

| Computer | Address/domain | Purpose |
| --- | --- | --- |
| Group 8 container | `192.168.10.78`, domain `8` | Base processing, mapping, custom dry-run navigation, RViz |
| Robot 4 System Computer | `192.168.10.64`, domain `64` | Shared hardware drivers, domain bridges, vendor motion bridge |
| Configured motion target | `192.168.1.120:43893` | Vendor UDP endpoint; not the group SSH address |

Group 7 shares this robot. Agree who controls it before any posture, mode or
movement action. No new Wi-Fi credentials or new motion bridge are required.

## 2. Physical preparation

1. Follow the manufacturer's/lab's normal power-on procedure; this guide does
   not invent power sequencing or charging instructions.
2. Check battery, stable level ground, and clear space around the body and legs.
3. Confirm group 7 has relinquished motion control.
4. Have an operator beside the robot with the previously tested STOP control.
5. Confirm the current posture visually. Do not assume yesterday's posture.
6. Leave it sitting for initial software checks. Standing is optional later.

If the app cannot control the robot normally, resolve that before ROS posture
tests. A STOP action may change posture; observe the result before continuing.

## 3. Terminal setup: every new group-container terminal

Open the group 8 desktop/terminal through your existing authorized connection.
The command prompt should be on `group8`, not `system`.

Run this block in **every** group terminal used below:

```bash
source /opt/ros/humble/setup.bash
source /home/user/ros2_ws/install/setup.bash
source /home/user/lite3_ws/install/transfer_interfaces/share/transfer_interfaces/local_setup.bash
source /home/user/lite3_ws/src/lite3_locomotion/.colcon/install/local_setup.bash
export ROS_DOMAIN_ID=8
```

Check once:

```bash
hostname
echo "$ROS_DOMAIN_ID"
ros2 pkg prefix lite3_launcher
ros2 pkg prefix lite3_locomotion
ros2 interface show transfer_interfaces/msg/MotionSimpleCMD
```

Expect `group8`, domain `8`, package prefixes for the university launcher and
your isolated custom installation, and fields `cmd_code`, `size`, `type`.

If a source file is missing or a package is not found: stop, check the exact
path and source order. Do not install a replacement bridge. These commands can
run from any folder; sourcing, not the current directory, selects packages.

## 4. Terminal S: verify the shared System Computer (read-only)

From a group terminal:

```bash
ssh user@192.168.10.64
```

Use your authorized lab login. For a new/changed SSH host key, verify its
fingerprint through a trusted lab record before accepting it. Never disable
host-key checking to work around a mismatch.

On the `user@system` prompt:

```bash
hostname
systemctl status lite3.service --no-pager
ps -eo pid,args | grep -E '[d]omain_bridge|[j]etson2motion'
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=64 ros2 param dump /motion_sender
```

Expected:

- Hostname `system`.
- `lite3.service` may say **active (exited)**: it launches tmux sessions. This
  alone is not an error; inspect its child processes.
- A running `jetson2motion` and bridges from domain 64 to domains 7 and 8.
- Sender target `192.168.1.120`, port `43893`, `use_sim_time: false`.

Optional read-only diagnostics on System:

```bash
ip route get 192.168.1.120
ping -n -c 2 -W 1 192.168.1.120
ROS_DOMAIN_ID=64 ros2 topic info /cmd_vel --verbose
ROS_DOMAIN_ID=64 ros2 topic info /simple_cmd --verbose
tmux list-windows -t lite3 -F '#{window_index}: #{window_name}'
```

The previous route used `eth0`, source `192.168.1.50`; ping responded. Expect
`motion_sender` subscriptions on `/cmd_vel` and `/simple_cmd`. Process IDs and
GIDs will change. Graph presence and ping do not prove actuator acceptance.

To inspect logs, first identify window names. Previously `robot` was window 3
and `domain_bridge` was window 4:

```bash
tmux capture-pane -p -t lite3:3.0 -S -120
tmux capture-pane -p -t lite3:4.0 -S -120
```

If those window numbers differ, substitute the current ones. Capture only
reads the panes. Do not send Ctrl+C to them or restart `lite3.service`.

If services are absent/unhealthy: save the status and logs and stop this
startup sequence. A shared-service restart affects both groups and is not part
of this checklist. Do not launch a second vendor bridge.

## 5. Terminal A: inventory group processes before launching anything

In a sourced `group8` terminal:

```bash
ros2 node list
ros2 topic list
ros2 topic info /livox/points --verbose
```

Look for existing `laserMapping`/Fast-LIO, `odom_relay`,
`pointcloud_to_laserscan`, `/slam_toolbox`, and `/lite3_autonomy/...` nodes.
Reuse healthy existing processes. Do not blindly run all startup commands over
an already running session. Duplicate node names/TF publishers are a warning
to identify the owner, not permission to kill shared processes.

Group `/cmd_vel` can be absent until a publisher appears: the domain bridge
creates forwarding dynamically. This does not by itself mean it is broken.
Do not publish a physical command merely to make the topic appear.

## 6. Terminal A: start base processing only, if absent

```bash
ros2 launch lite3_launcher lite3_bringup.launch.py
```

Leave the terminal running. This local include starts Fast-LIO, odometry relay
and pointcloud-to-scan processing; hardware drivers remain on System.

Expect `/cloud_registered_body`, `/odom`, `/scan` and `odom -> base_link`.
It is a long-running process, not a command that finishes after startup.

If the launch file cannot be located:

```bash
find /home/user/ros2_ws/install/lite3_launcher/share/lite3_launcher/launch -name lite3_bringup.launch.py -print
```

Check the launcher installation/source overlay. Do not build or overwrite
university software as an automatic troubleshooting step.

## 7. Terminal B: start SLAM only, if absent

Run the terminal setup block, then:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  use_sim_time:=false \
  slam_params_file:=/home/user/ros2_ws/src/lite3_launcher/config/nav2_params.yaml
```

Leave it running. This uses the existing university `slam_toolbox` parameters:
`odom`, `map`, `base_link`, `/scan`. Expect `/map` and `map -> odom`.
This starts a new mapping session, not automatic restoration of yesterday's
SLAM pose graph. Saved-map localization is a different workflow.

**Correction to earlier chat instructions:**
`lite3.launch.py slam:=True navigation:=False` does not start SLAM in this
launcher: its SLAM include lives inside the disabled navigation include.
The separate base + SLAM launches above avoid this issue and avoid starting
the university's physical-output Nav2/mux chain alongside custom dry-run Nav2.

## 8. Terminal C: verify live data before starting autonomy

Run the setup block, then run these individually:

```bash
timeout 8s ros2 topic hz /scan
timeout 8s ros2 topic hz /odom
timeout 6s ros2 run tf2_ros tf2_echo odom base_link
timeout 6s ros2 run tf2_ros tf2_echo map base_link
timeout 6s ros2 topic echo /map nav_msgs/msg/OccupancyGrid \
  --once --no-arr --qos-durability transient_local
```

`timeout` deliberately ends each diagnostic; exit code 124 at its deadline is
normal for these continuous tools. Alternatively Ctrl+C only the diagnostic.

Expect repeated scan/odom data (previously scans were approximately 10 Hz),
updated transforms, and a map with positive dimensions/resolution. An initial
TF discovery warning followed by transforms can be normal. Persistent missing
frames are not.

Map metadata alone cannot prove mapping succeeded. In RViz confirm that some
cells are known free/occupied; an entirely unknown (`-1`) map is not ready.

If a check fails, use the troubleshooting table before proceeding.

## 9. Terminal D: start custom Nav2 in dry-run only, if absent

Run the setup block, then:

```bash
ros2 launch lite3_locomotion autonomy.launch.py output_mode:=dry-run
```

Leave it running. Expect planner, controller, BT navigator, lifecycle manager,
disarmed guard and idle exploration coordinator. Lifecycle nodes should become
active. A log about creating a bond timer is lifecycle monitoring, not an
error and not a process you need to wait to finish.

This launch does not start mapping, a robot bridge, posture changes or Auto
mode. Do not also start the full university navigation stack.

Check from Terminal C:

```bash
ros2 lifecycle get /lite3_autonomy/controller_server
ros2 lifecycle get /lite3_autonomy/planner_server
ros2 lifecycle get /lite3_autonomy/bt_navigator
timeout 5s ros2 topic echo /lite3_locomotion/guard_status \
  --once --qos-durability transient_local
```

Expect all three lifecycle states `active`, `output_mode: dry-run`, and initially
`armed: false`. Healthy status requires current scan and TF, not just active
lifecycle nodes.

## 10. Terminal E: visualize

Run the setup block, then:

```bash
rviz2
```

Use fixed frame `map`. Add:

- Map: `/map`, durability Transient Local.
- LaserScan: `/scan`, reliability Best Effort.
- TF.
- Path: `/lite3_autonomy/plan`.
- Optional costmaps under `/lite3_autonomy/...` and MarkerArray
  `/lite3_locomotion/exploration_markers` when a preview publishes it.

Expect the scan to line up with obstacles and a small initial map. RViz is a
viewer, not the source of mapping/navigation data. A stand movement may change
LiDAR visibility; let TF/scan settle and recheck their alignment.

## 11. Terminal C: run a bounded dry-run exploration mission (optional)

Check that the guard reports **dry-run** first. Then:

```bash
ros2 run lite3_locomotion autonomy_control start
```

This arms the dry-run guard **and starts frontier exploration**. It is not a
stand command, keyboard teleop, or a two-second forward test.

Inspect individually:

```bash
timeout 5s ros2 topic echo /lite3_locomotion/guard_status --once --qos-durability transient_local
timeout 5s ros2 topic echo /lite3_locomotion/exploration_status --once
timeout 5s ros2 topic echo /lite3_locomotion/guarded_test_cmd_vel --once
```

Expected: dry-run status and, if a safe reachable frontier exists, planning and
test velocity output. Nothing from this dry-run chain should move the robot.
A stationary robot can trigger Nav2's progress timeout, approximately 15 s in
the current configuration. The default guard session is bounded at 60 s.

If no frontier is eligible, dry-run may stop with zero goals. That is a valid
rejection, not a reason to reduce clearance or enlarge bounds to force motion.
Do not remap test output to `/cmd_vel`.

Stop explicitly:

```bash
ros2 run lite3_locomotion autonomy_control stop
```

Expect stop/cancellation service responses and a disarmed state, often reason
`operator_stop`. Repeated echo output continues until Ctrl+C; it is not stuck.

## 12. Optional: reproduce the verified physical stand milestone

This section causes **physical posture movement**, unlike steps 6–11. It is
not necessary for mapping or dry-run navigation. Prefer the manufacturer's
normal posture control if you only need to stand the robot.

Only proceed when all are true: group 7 has relinquished control; no navigation
mission or other command source is active; the robot is visually confirmed
sitting; everyone is clear; the operator is beside it with tested STOP ready;
and its normal app controls work. Stop the dry-run mission first.

First open an observer on System (Terminal S):

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=64 timeout 30s ros2 topic echo /simple_cmd \
  transfer_interfaces/msg/MotionSimpleCMD --once \
  --qos-reliability reliable --qos-durability volatile
```

While it waits, in a sourced group terminal publish **once**:

```bash
ROS_DOMAIN_ID=8 timeout 8s ros2 topic pub --once --keep-alive 2 \
  --qos-reliability reliable --qos-durability transient_local \
  /simple_cmd transfer_interfaces/msg/MotionSimpleCMD \
  '{cmd_code: 0x21010202, size: 0, type: 0}'
```

Expected local output: exactly `publishing #1`. Expected System observation:

```text
cmd_code: 553714178
size: 0
type: 0
```

The operator must confirm the robot stands. The observer proves ROS-domain
delivery, not motor acceptance. This exact command was observed to stand the
sitting robot in this session. It is a **toggle**: sending again may sit it.
Never loop or automatically retry it. Publication failure/timeouts are not
permission to resend without observing posture and delivery.

If nothing happens, do not follow it with walking commands. Record the observer
output and inspect the System `robot`/`domain_bridge` panes. Check current mode,
STOP state and app control. Transient-local command publishers/bridges may
retain messages: do not restart command consumers to force a retry.

For reference, `553716739` (`0x21010C03`) is Auto mode, **not** stand/sit.
Changing Auto mode is not required for dry-run and is not part of this routine.
The installed `lite3_auto`/`lite3_manual` aliases are physical mode commands,
not navigation launch commands; do not execute them casually while diagnosing.

## 13. Troubleshooting: diagnose, do not duplicate or bypass

| Symptom | Check / action | Expected recovery |
| --- | --- | --- |
| `ros2` missing or package not found | Repeat step 3; check source paths and `ros2 pkg prefix` | Correct underlay/overlay selected; no reinstall needed |
| SSH to `.64` fails | `ping -n -c 2 -W 1 192.168.10.64`; verify authorized network/login/host key | System reachable; no scanning other hosts or disabling trust checks |
| System service says `active (exited)` | Inspect child processes and tmux logs | Running drivers/bridges despite exited startup wrapper |
| `/cmd_vel` absent in group 8 | Bridge is created when a source publisher appears | Not by itself a fault; do not inject a motion command as a discovery test |
| `/scan` absent | Inspect base terminal, `/cloud_registered_body`, `/livox/points` endpoints | Single working cloud-to-scan pipeline |
| `/livox/points` reliability warning | Use verbose endpoint info to identify the incompatible subscriber; RViz should use Best Effort | Fix the specific consumer; warning is not evidence the posture command failed |
| `odom` frame does not exist persistently | Check Fast-LIO and odom relay from step 6, domain and data | Live `odom -> base_link`; never fabricate a static odometry transform |
| `map` frame/map absent | Verify one SLAM instance and step 7, scan and odometry | Live `map -> odom` and `/map`; `navigation:=False` combined launch does not start SLAM |
| RViz map invisible | Fixed frame `map`, map `/map`, Transient Local; confirm actual map publisher | Visible map; QoS/display errors corrected |
| Entire map unknown | Check finite scan returns, sensor alignment, TF and SLAM logs | Known free/occupied cells; do not explore an all-unknown map |
| `waiting_for_map` | Allow several update cycles; verify map publisher and subscription QoS | Map received; persistent waiting requires diagnosis |
| `no_eligible_frontiers...` | Inspect free space, boundary, footprint and map; do not reduce safety margins blindly | Mission remains stopped if no valid safe goal exists |
| Guard `operator_stop` / disarmed | Expected after stop; fix health issues before explicit dry-run start | Start only a reviewed dry-run mission |
| Guard rejects scan/TF | Verify freshness, frames, coverage and timestamps; same ROS time basis | Healthy readings without disabling freshness checks |
| Nav2 progress failure in dry-run | Robot is stationary while Nav2 expects progress | Expected limitation; not permission to enable physical output |
| Duplicate nodes or occupied resources | Identify existing launch owner; reuse healthy instance | One mapping/TF chain and one intended navigation stack |
| Posture command echoes but no movement | Echo is not actuator acknowledgment; inspect driver/state without repeating | Root cause identified before another physical attempt |

Current footprint, clearance and velocity settings are placeholders pending
measurement. A converted 2D LiDAR scan is not a drop-off/terrain detector and
does not guarantee collision prevention.

## 14. End-of-session routine

1. Stop your mission with `autonomy_control stop`; inspect disarmed status.
2. If you want today's map for later localization, save it **before** stopping
   SLAM. From a sourced group terminal, use a new filename each session:

   ```bash
   ros2 run nav2_map_server map_saver_cli -f /home/user/lite3_ws/session_map_YYYYMMDD_HHMM
   ```

   Replace the date/time placeholder yourself to avoid overwriting a map.
   Expect a YAML and image file. Saving an occupancy map does not preserve the
   complete SLAM pose graph or automatically restore localization tomorrow.
3. If physical control was used, use the manufacturer's normal stop/rest/mode
   procedure and visually confirm its state. Software stop is not an emergency
   stop; do not assume killing a terminal stops hardware.
4. Ctrl+C only the group launch processes **you started**, then close RViz.
   Closing an SSH terminal does not stop the shared tmux services.
5. Leave System services running; tell the other group you are finished.

## 15. Completion checklist and next development task

### Follow-up: failed walking attempt and corrected test runner

A later 1 s forward commissioning attempt produced no observed walking. Its
System log confirmed 20 nonzero callbacks and successful forward UDP sends,
but also a 1.436 s last-forward-to-first-zero gap caused by separate CLI
publishers. **Do not repeat that two-process sequence.** The replacement
[single-process pulse runner](commissioning_pulse.md) has passed isolated timing
tests; it has not yet been used for another physical attempt. Current
Navigation-mode acceptance and the cause of no walking remain unverified.

- [ ] Correct group/System hosts and domains confirmed.
- [ ] Existing shared drivers/motion bridge healthy; no duplicate bridge.
- [ ] Live scan, odometry, TF and map with known cells.
- [ ] Custom Nav2 active and guard explicitly dry-run.
- [ ] Dry-run start/stop understood; rejection reasons preserved.
- [ ] Optional single posture toggle observed in domain 64 and confirmed physically.

At this point you are back at the demonstrated milestone. Physical autonomous
walking is **not** completed by these steps. Next implement a reviewed guarded
university-bridge profile, validate its obstacle/stale-data/stop behavior in
isolation, then commission a bounded forward-and-stop test before any goal or
frontier exploration. Do not run the recovery candidate or separate custom UDP
sender alongside the already working university motion bridge.

Related implementation details: [autonomy stack](autonomy_stack.md),
[guarded adapter](../lite3_locomotion/guarded_adapter.py),
[control client](../lite3_locomotion/autonomy_control.py).
