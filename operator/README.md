# Lite3 navigation: operator guide

> 2026-11-21: the M0-M9 physical test suite, Test 4 and walk.sh were removed; autonomous roaming
> (`operator/roam.sh`) is the only motion workflow. Copies of the removed files are in `.baseline/`.

This guide covers starting, checking, testing (Test 4) and stopping the Lite3
navigation software without Claude Code. All of the procedures in this guide
are **non-actuating**. Navigation runs in *dry* mode: Nav2's commands go to
`/lite3_locomotion/dry_cmd_vel`, which nothing on the robot listens to. The
robot does not move. Keep the robot's emergency stop within reach anyway.

Supervised tests that **move** the robot (M0–M9, from basic walking to the
obstacle course) are described separately in [PHYSICAL_TESTS.md](PHYSICAL_TESTS.md).
They need `start_navigation.sh --physical` plus a typed confirmation per test.
Autonomous roaming (M10, independent of M0–M9) is in §0.1 below.

## 0.1 M10 autonomous roaming (robot MOVES on its own): `operator/roam.sh`

M10 explores **without goals** until you stop it. It is independent of M0–M9: no earlier test has
to pass, because it runs its own readiness checks before anything moves. Those checks are:
telemetry, Nav2/SLAM/LiDAR, the mux wiring, the stop boundary against the live LiDAR delay, zeros
reaching the robot with the dead-man locked while idle, then stand up and verify standing. The
dry mode is the default.

```bash
operator/roam.sh start                    # dry: readiness + where it WOULD go; nothing is published
operator/roam.sh start --sim              # rehearsal against the software stand-in (no robot)
operator/roam.sh start --physical         # starts navigation (type ARM) if needed, checklist, type ROAM
operator/roam.sh start --physical --geofence 2.5 --speed slow --speed-limit 0.08
operator/roam.sh plan --max-speed 0.2      # effective limits, stop zones, turning clearance (no motion)
operator/roam.sh start --physical --max-speed 0.2   # NOT validated above 0.10: see `plan` first
operator/roam.sh start --physical --strategy frontier   # old behaviour: frontier/patrol goals
# from any other terminal while it runs:
operator/roam.sh status                   # state, speed, goal, map size, clearance, LiDAR, watchdog
operator/roam.sh pause                    # cancel goal, zeros, dead-man locked (robot stands)
operator/roam.sh resume                   # re-checks sensors and standing first
operator/roam.sh speed normal             # slow | normal | fast | up | down
operator/roam.sh stop                     # cancel goal, zeros, verify rest, write the report
operator/roam.sh modes                    # what the speed modes are (and why not gait modes)
```

Ctrl+C in the roaming terminal and `operator/abort_physical.sh` also stop it. `stop_navigation.sh`
stops it first. **None of these is an emergency stop: the handheld remote is.**

| Item | Value |
| --- | --- |
| What it does | **Default `--strategy scan-walk`:** walk straight ahead when that corridor is open, or have Nav2 steer into a visible opening. If no walk goal is available, look up to 60° to either side in 30° steps. Each new SLAM map can end the scan early. The runner pauses if both sides have been searched or the terrain lock is active. It ends a forward approach when the live obstacle range reaches the walking standoff. Goals are placed so the next spin fits: nothing known within the stop zone (collision_monitor polygon / terrain stop-ahead, whichever is larger) of the goal, otherwise the lock would hold even turning in place. Roaming starts navigation with `--clearance 0.05 --max-speed V` (`roam.sh start --max-speed V`, default 0.50): at 0.10 a **0.22 m stop boundary from the nose** (terrain proximity stop 0.20 m), so a walk goal needs about 1.3 m of open floor across. Above 0.10 the collision_monitor gets two polygons: a STOP polygon sized for 0.10 m/s (the only one that also stops turning) and a SLOWDOWN polygon sized for the full stop from V that scales every command (turning too) to <= 0.10 m/s while anything is inside it. A single V-sized stop polygon left the robot unable to turn away from anything within ~1 m (simulation: 0.30 m/s never moved; with the slowdown polygon 0.30 and 0.50 roam normally). The terrain backup lock stays at 0.20 m, so above 0.10 it backs up the 0.10 m/s tier, not a full-speed stop. Above 0.10 the `fast` mode becomes V. Higher `--max-speed` stacks have a larger stop zone and need more open space (at 0.5 m/s about 3.2 m across); roaming warns at start when the open circle needed exceeds 2 m. 5 consecutive spins/walks without a way on **pause** roaming. `--strategy frontier`: the earlier frontier goals with patrol fallback (needs an already detailed map). |
| Point cloud | Goals are vetoed where the Nav2 global costmap is high. That costmap includes the LiDAR point-cloud hazards from `terrain_monitor`, so tables, low objects and drops the 2D map misses are avoided as targets. Raw `/livox/points` rate and size and Fast-LIO's `/cloud_registered_body` are monitored, so a blind LiDAR stops roaming even if the derived status still looks healthy. |
| Speed modes | M10 starts in normal at 0.50 m/s, capped by the session limit. Slow requests 0.05 m/s and fast can use the session maximum. The navigation interface accepts up to 1.0 m/s forward. Stopping-distance and freshness checks also apply. |
| Gait modes | **Not used.** This deployment has no verified gait-switch command or gait feedback: the vendor bridge has none, and `robot_gait_state` is dropped. The robot keeps the gait it stands up in. |
| Stops automatically on | Stale or invalid telemetry; tilt; handheld remote input; command path changed; robot-side speed above the mode or the limit; reverse; actual (leg-odometry) speed far above the command; map-pose geofence; obstacle < `--abort-clearance` (0.08 m) from the footprint; LiDAR/terrain/hazard data stale or slow; raw LiDAR empty or stale; Fast-LIO lost; SLAM map not updating; watchdog lost or tripped; `--max-minutes` (≤ 120). |
| Independent watchdog | A separate process with its own ROS node. It checks telemetry, tilt, raw LiDAR, robot-side speed, the leg-odometry geofence (+0.75 m) and the runner's heartbeat. On a fault it interrupts the runner (which then stops the robot), kills it if needed (the twist_mux dead-man then locks within 0.5 s), and commands zeros. The runner in turn stops if the watchdog's heartbeat is lost. |
| Results | `log/operator/run-<start>/roam/<time>-<mode>/report.md`: outcome, stop reason, goals, distance, mapped area, minimum clearance, events, plus the readiness report. |

Before a physical session: clear the geofence area of people, stairs and drops. Hold the remote.
Watch the robot the whole time.

## 1. Which computer does what

| Computer | Address (verified 2026-11-16) | Role | What you run there |
| --- | --- | --- | --- |
| **group8 container** (the student VM, an LXC on the university server) | `192.168.10.78`, user `user`, hostname `group8` | Runs the navigation stack, the tests and these scripts (ROS domain **8**) | **Everything in this guide** |
| Robot system computer | `192.168.10.64` | University-managed: Livox LiDAR drivers, `motion_sender`/`motion_receiver`, `sensor_checker`, and the domain bridges to groups (ROS domain **64**) | Nothing. It only has to be powered on and running |
| Robot motion controller | `192.168.1.120` | Vendor locomotion firmware (UDP target of `motion_sender`) | Nothing |
| RViz terminal | `192.168.10.108` | Visualisation PC | Not used by these scripts |

The scripts check that the robot side is up: they look for `lite3_domain_bridge_8`,
robot telemetry and LiDAR in domain 8.

## 2. Log in and go to the project

```bash
ssh user@192.168.10.78
cd ~/lite3_ws
```

You need a lab connection that can reach `192.168.10.78`. VS Code Remote-SSH to
the same address also works; use its terminal.

**One-time setup, and after any code change:**

```bash
operator/build.sh
```

It builds `lite3_locomotion` and `lite3_movement_tests` into
`~/lite3_ws/install`. It never touches the university workspace `~/ros2_ws`.

## 3. Daily procedure

```bash
cd ~/lite3_ws
operator/preflight.sh            # read-only checks (about 12 s)
operator/start_navigation.sh     # starts everything in the background (about 30-60 s)
operator/roam.sh start --physical --max-speed 0.9   # type ARM, then ROAM
operator/stop_navigation.sh      # always run this when finished
```

You don't need to source anything. Every script sets up ROS Humble, `~/ros2_ws`,
`~/lite3_ws` and `ROS_DOMAIN_ID=8` itself.

### 3.1 Preflight: `operator/preflight.sh`

The preflight is read-only. It checks:

- the ROS environment and the required packages;
- the university domain bridge;
- that **nobody publishes `/cmd_vel`**;
- the static TFs;
- robot telemetry (`/joint_states`, `/imu/data`, `/leg_odom2`): it needs at least
  100 Hz with gaps under 0.25 s;
- direct mode checks the native Livox scan alongside joint telemetry. A local converter produces the terrain cloud from the same scan. Legacy bridged preflight checks both LiDAR streams (`/livox/points`, `/livox/lidar`): it needs at least 8 Hz;
- IP-fragment reassembly health while the LiDAR streams flow.

Each line is `OK`, `NOTE` (advice only), `WARN` or `FAIL`. Every non-OK line is
followed by an `->` line saying what to do. The overall result is one of:

| Result | Meaning |
| --- | --- |
| `READY` | Go ahead |
| `DEGRADED` | Warnings; results would be unreliable |
| `NOT READY` | Do not start |

Each run is saved as `log/operator/preflight-*.json`.

### 3.2 Start: `operator/start_navigation.sh`

1. Runs the preflight again. It refuses on `NOT READY`, and on `DEGRADED` unless
   you pass `--accept-degraded`. That flag is only for inspecting the stack; test
   results from a degraded link are not valid.
2. Refuses to start a second copy, whether this workflow is already running or
   navigation nodes from another terminal or user exist in domain 8.
3. Starts the existing launch file `lite3_locomotion navigation.launch.py` in the
   background, with its **default dry output**. That brings up:
   - the university `lite3_bringup` (Fast-LIO, `odom_relay`, `pointcloud_to_laserscan`);
   - the university `lite3_navigation` (SLAM toolbox plus Nav2, using
     `params/nav2_lite3.yaml`);
   - `terrain_monitor`;
   - `twist_mux` with the proximity-stop lock.
4. Waits until all of the following hold, then prints `NAVIGATION READY (dry,
   non-actuating)`:
   - all six Nav2 lifecycle nodes are active;
   - `/odom`, `/scan`, `/lite3_terrain/stop` and `/lite3_terrain/status` are
     flowing at about 10 Hz;
   - the `NavigateToPose` action exists;
   - **`/cmd_vel` has no publisher, `twist_mux` publishes only the dry topic, and
     nothing consumes the dry topic**.

   If startup fails, it shows the log errors, stops the partial stack and tells
   you what to do.

You can close the terminal afterwards. The stack keeps running until
`operator/stop_navigation.sh`.

### 3.3 Status at any time: `operator/status.sh`

```bash
operator/status.sh               # one check
operator/status.sh --watch 10    # repeat every 10 s (Ctrl+C to leave; this does not stop anything)
```

It shows the Nav2 lifecycle, data rates and the three NON-ACTUATING checks.

## 5. Aborting and stopping

| Situation | Action |
| --- | --- |
| Abort Test 4 (the terminal it runs in) | **Ctrl+C**. The goal is cancelled and a partial report saved (`ABORTED`) |
| Abort Test 4 from another terminal | `operator/abort_test4.sh` |
| Stop everything | `operator/stop_navigation.sh` |
| Robot moves unexpectedly (should be impossible here) | **Robot emergency stop / handheld remote first**, then `operator/stop_navigation.sh` |

`operator/stop_navigation.sh` does the following:

1. Stops Test 4 if it is running.
2. Cancels all `NavigateToPose` goals.
3. Sends SIGINT to the processes **this workflow started** (one process group).
   It escalates to SIGTERM, then SIGKILL, after 10 s. The Nav2 container usually
   needs this; it typically takes about 15–20 s.
4. Verifies that none of them remain and that the ROS graph is clear.

It never touches university services or other users' processes, and it keeps all
logs.

## 6. Logs and results

Each start creates `~/lite3_ws/log/operator/run-YYYYMMDD-HHMMSS/`, containing:

| File | Contents |
| --- | --- |
| `launch.log` | All node output |
| `ros_logs/` | ROS log files |
| `preflight.json`, `ready.json` | Startup checks |
| `test4-precheck.json` | Status check before each test |
| `test4/<time>/summary.md` | Test 4 result table (read this first) |
| `test4/<time>/report.json` | All checks plus recorded data |

```bash
ls -td ~/lite3_ws/log/operator/run-* | head -1                                  # newest run
cat "$(ls -td ~/lite3_ws/log/operator/run-*/test4/* | head -1)/summary.md"      # newest Test 4 result
tail -f "$(ls -td ~/lite3_ws/log/operator/run-* | head -1)/launch.log"          # follow node output
```

`log/` is not tracked by git. Copy results elsewhere if you want to keep them
long-term.

## 7. Restarting after a failure

```bash
operator/stop_navigation.sh      # safe to run any time; finds leftovers even if a terminal or script crashed
operator/preflight.sh            # fix every FAIL it reports
operator/start_navigation.sh
```

If `start_navigation.sh` says processes from an earlier run are still alive, run
`operator/stop_navigation.sh` first. If stop reports that a process group
survived, it prints the exact `kill -KILL -- -<pgid>` command for that group.
Those are only this workflow's processes.

## 8. Troubleshooting

| Symptom | Likely cause | What to do |
| --- | --- | --- |
| `university domain bridge ... absent` and/or robot telemetry 0 Hz | Robot or system computer (`192.168.10.64`) off, or the lab bridge not running | Check that the robot is powered and booted. Ask the lab to check the domain bridge. Nothing to fix on group8 |
| Telemetry OK but `LiDAR ... Hz` low | IP-fragment overload on group8 (see 8.1), or network load | Run the preflight and read the `IP fragment reassembly` line. If it shows failed reassemblies, apply 8.1. If it shows 0 failures and LiDAR is still low, the bottleneck is upstream: the robot side also runs bridges for other groups (`lite3_domain_bridge_group_a` and `_group_b`), so ask the lab |
| Preflight `robot telemetry ... max gap 0.4 s` on all three topics at once | A brief network stall (seen once on 2026-11-16; the next preflight was clean) | Re-run `operator/preflight.sh`. If it keeps happening the lab network is unstable, so tell the lab; the movement tests would refuse to run too |
| `data /odom` low after start | Same as above: Fast-LIO only publishes `/odom` when LiDAR arrives | Stop, preflight, fix the LiDAR, start again |
| `Nav2 ... unavailable/inactive` | Startup error | Read the errors that `start_navigation.sh` printed and `launch.log`, then follow section 7 |
| `navigation nodes already exist ... not started by this workflow` | Someone started the stack in another terminal or account | Find who. Don't run two stacks in domain 8 |
| `NON-ACTUATING: /cmd_vel publishers` FAIL | Another program is commanding the robot | **Stop**. Use the e-stop if the robot moves. Find that program before continuing |
| Proximity stop on a "clear" floor (`hazard within 0.45 m`) | Something really is within 0.45 m around the robot: a person, a chair, a desk edge up to 0.75 m high. On 2026-11-16 an object about 0.5 m to the robot's left caused exactly this | Clear the surroundings. The monitor ignores only the robot's own body box (±0.40 m by ±0.25 m) |
| Test 4 `INCONCLUSIVE` | The baseline scene already had obstacles where the test places objects | Clear the area and run the test again |
| Shutdown log shows `odom_relay.py ... process has died` | University `odom_relay` exits with an error on shutdown | Harmless |

### 8.1 LiDAR fragment fix (Linux IP fragment reassembly)

**What happens.** Each LiDAR message is large: 401 KB on `/livox/lidar` and 442
KB on `/livox/points`, both at 10 Hz, about 8.5 MB/s together. The navigation relay now requests only the native Livox scan (about 4 MB/s) and converts it to PointCloud2 on group8. Other subscribers to either source topic can still add traffic. Each one arrives
as roughly 45 IP fragments per 64 KB datagram. The kernel's default reassembly
cache is 4 MB, and an incomplete datagram is held for 30 s.

On 2026-11-13 the cache sat full (4,197,400 bytes, against a 4,194,304 limit),
with 4.27 M reassembly failures. LiDAR fell to 0.4–2 Hz while the small 200 Hz
telemetry was unaffected.

**How the preflight detects it:** the `IP fragment reassembly` line. It counts
successful and failed reassemblies while both LiDAR streams flow, and how full the
cache is.

**The fix.** Run on group8. It needs your sudo password; the scripts never store
or ask for it.

```bash
sudo sysctl -w net.ipv4.ipfrag_high_thresh=67108864 net.ipv4.ipfrag_low_thresh=50331648 net.ipv4.ipfrag_time=3
```

That sets a 64 MB cache, starts evicting at 48 MB, and drops incomplete
datagrams after 3 s. A datagram's fragments normally arrive within milliseconds.

Sizing: 8.5 MB/s of payload is about 17 MB/s of kernel buffer. Holding three
seconds of it costs about 51 MB, which is still under 64 MB. That is the
worst case, where nothing ever completes.

**Scope.** The change affects only this container's network namespace, not the
university server or other containers. The evidence, checked on 2026-11-16:

- group8 is an **unprivileged** LXC: `/proc/self/uid_map` maps container root to
  host uid 100000.
- It has its own network namespace and veth interface (`eth0@if7`).
- `net.ipv4.ipfrag_secret_interval` and most `net.core.*` keys are absent here.
  Those are registered only in the host's initial namespace, which shows that
  the visible `ipfrag_*` keys are this namespace's own copies.

The cost is up to 64 MB of kernel memory for this container's reassembly. If the
lab administers the host, they can confirm on the host that
`sysctl net.ipv4.ipfrag_high_thresh` still shows its own value.

**Verify:**

```bash
sysctl net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_low_thresh net.ipv4.ipfrag_time
grep FRAG /proc/net/sockstat
operator/preflight.sh
```

`grep FRAG` shows the bytes currently held. The preflight line
`ipfrag sysctls (preventive)` turns `OK`.

**Revert** (these are the kernel defaults that were in place before):

```bash
sudo sysctl -w net.ipv4.ipfrag_low_thresh=3145728 net.ipv4.ipfrag_high_thresh=4194304 net.ipv4.ipfrag_time=30
```

**Lifetime.** The setting is lost when the container restarts. The values were
back to defaults on 2026-11-16 after a restart. To make it permanent, if the lab
agrees, create a file and reload:

```bash
printf 'net.ipv4.ipfrag_high_thresh = 67108864\nnet.ipv4.ipfrag_low_thresh = 50331648\nnet.ipv4.ipfrag_time = 3\n' | sudo tee /etc/sysctl.d/90-lite3-ipfrag.conf
sudo sysctl --system
```

To undo the permanent version, delete that file and run the revert command
above.

The longer-term fix sits on the robot side: configuring its DDS to avoid
IP-fragmented messages. That is university-owned configuration; raise it with the
lab.

## 9. What the dry mode guarantees, and what it doesn't

**Guaranteed by these scripts:**

- Navigation's only path to the robot, `twist_mux`, is launched with its output
  on `/lite3_locomotion/dry_cmd_vel`.
- The launch file refuses `/cmd_vel` unless `allow_robot_motion:=true` is also
  given, and these scripts never pass it.
- Startup, the status check and Test 4 all verify that `/cmd_vel` has no
  publisher.
- Test 4 also watches the robot-side `/cmd_vel` (domain 64) and aborts if a single
  message appears.

**Not covered:** other people or programs commanding the robot directly, and the
handheld remote. Those are why the e-stop stays within reach.

Physical navigation tests are a separate, explicitly authorized procedure, run
through `lite3_movement_tests`. They are not part of this guide.


### System-host FAST-LIO

The system container at 192.168.10.64 runs the calibrated Mid-360 mapper on ROS domain 64 alongside the sensor drivers. The private Group 8 bridge carries odometry and the system-host LaserScan. In system mode, the Group 8 relay exposes that scan as /scan and the odometry relay publishes the base transform. The terrain monitor still gets height from the separate native Livox stream and local PointCloud2 converter.

The system-host files are in ~/group8_lio. After a system-host reboot, start the mapper there with
`tmux new-session -d -s group8_lio "cd ~/group8_lio && ./start.sh > run.log 2>&1"`.
The directory contains the fastlio_mapping executable and mid360.yaml from the group8 ROS installation with the same relative paths as the operator/system_lio/start.sh script expects. The outputs bridge uses outputs.yaml from the same directory. The scan converter requires the compatible Humble pointcloud_to_laserscan_node and libpointcloud_to_laserscan.so in the deployment directory. Copy the scan YAML and rgb_preview.py from operator/system_lio. The existing JPEG stream is sampled at up to 5 fps into /group8_perception/rgb/image/compressed for the Group 8 preview.

Group8 defaults to the system mapper. For local fallback, stop group8 navigation, export LITE3_LIO_SOURCE=local in every operator shell for that session, then start navigation. The private system-host topics can coexist with local mapping. Use the regular readiness checks before physical motion.
