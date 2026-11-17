# Lite3 navigation: operator guide

This guide covers starting, checking, testing (Test 4) and stopping the Lite3
navigation software without Claude Code. All of the procedures in this guide
are **non-actuating**. Navigation runs in *dry* mode: Nav2's commands go to
`/lite3_locomotion/dry_cmd_vel`, which nothing on the robot listens to. The
robot does not move. Keep the robot's emergency stop within reach anyway.

Supervised tests that **move** the robot (M0–M9, from basic walking to the
obstacle course) are described separately in [PHYSICAL_TESTS.md](PHYSICAL_TESTS.md).
They need `start_navigation.sh --physical` plus a typed confirmation per test.
Autonomous roaming (M10, independent of M0–M9) is in §0.1 below.

## 0. Quick reference: walk and go around obstacles (robot MOVES)

Full details are in [PHYSICAL_TESTS.md](PHYSICAL_TESTS.md). Keep the handheld remote/e-stop in
your hand.

```bash
operator/preflight.sh                                         # READY? also measures LiDAR delay
operator/start_navigation.sh --physical --clearance 0.40      # type ARM; prints the stop boundary
operator/status.sh                                            # chain: twist_mux -> collision_monitor -> /cmd_vel
operator/run_physical.sh --physical --phases M0,M1,M2         # once per start: path, motion, stopping
operator/run_physical.sh --physical --phases M3               # box 0.20 m ahead: must not move
operator/run_physical.sh --physical --phases M4               # walk until a chair (anywhere ahead) stops it
operator/run_physical.sh --physical --phases M5               # Nav2 straight line
operator/run_physical.sh --physical --phases M7               # Nav2 goes AROUND a box (no emergency stop)
operator/walk.sh --distance 3.0 --physical                    # walk 3 m ahead, going around obstacles
operator/abort_physical.sh                                    # commanded stop from another terminal
cat "$(ls -td log/operator/run-*/physical/*/* | head -1)/report.md"    # latest result
operator/stop_navigation.sh                                   # always at the end
```

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
| What it does | Frontier goals on the live SLAM map inside the geofence; when none are left, patrol goals in known free space far from recent goals. Nav2 plans around obstacles and replans. Failed goals are blocked and replaced; 5 consecutive failures **pause** roaming. |
| Point cloud | Goals are vetoed where the Nav2 global costmap is high. That costmap includes the LiDAR point-cloud hazards from `terrain_monitor`, so tables, low objects and drops the 2D map misses are avoided as targets. Raw `/livox/points` rate and size and Fast-LIO's `/cloud_registered_body` are monitored, so a blind LiDAR stops roaming even if the derived status still looks healthy. |
| Speed modes | `slow` 0.05 m/s (the default), `normal` 0.08, `fast` 0.10 (with yaw 0.15 / 0.20 / 0.20 rad/s). Change them with `--modes "slow=0.04,fast=0.09"`. Each mode is capped by `--speed-limit` (default: the stack's `--max-speed`, which is 0.10 unless you start navigation with a higher one; hard ceiling 0.5). **Only speeds up to 0.10 m/s are validated.** For a higher session limit start navigation with `--max-speed V` first; the stopping distance is then assumed, not measured (PHYSICAL_TESTS.md §1.1). Enforced in three places: the Nav2 velocity smoother (set and read back), the MPPI speed limit, and robot-side command checks. |
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
operator/run_test4.sh            # five-minute Test 4, follow the on-screen phases
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
- both LiDAR streams (`/livox/points`, `/livox/lidar`): it needs at least 8 Hz;
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

## 4. Test 4: `operator/run_test4.sh`

**Goal.** Prove, with the real sensors and real objects, that obstacle detection,
the costmaps, Nav2 and the proximity stop work together. Nav2 gets a goal 2.5 m
ahead every 30 s. What it *would* command is recorded but never reaches the
robot. The robot may stay lying down.

**Before starting:**

- Clear the area: nothing within **0.6 m all round** the robot (people included)
  and **2.5 m ahead**.
- Prepare a box at least 20 cm tall, and a low object 5–8 cm tall (for example a
  thick book).

Measure distances from the robot's **front edge (nose)**. The test converts them:
body centre = nose + 0.30 m.

| Phase | Time | You do | Expected |
| --- | --- | --- | --- |
| P1 CLEAR | 0–60 s | Stay outside the cleared area | No stop. Nav2 issues commands (≤ 0.10 m/s) |
| P2 BOX FAR | 60–120 s | Box straight ahead, front face **1.2 m** from the nose | Obstacle about 1.5 m from centre. No stop |
| P3 BOX NEAR | 120–180 s | Move the box to **0.25 m** from the nose | Stop activates ("hazard … ahead"). Nav2 commands are blocked at the mux |
| P4 LOW OBJECT | 180–240 s | Remove the box. Low object at **1.0 m** from the nose | Detected as a step (or an obstacle if over 10 cm) at about 1.3 m |
| P5 WALK ACROSS | 240–300 s | Remove it. Walk across 1–1.5 m in front, then leave | Brief obstacle, then clear |

The screen prints the instructions at each phase change (with a terminal bell)
and one status line per second, for example:

```
[  135/300s] P3 | odom 10.0 Hz terrain 10.0 Hz | obstacle 0.55 step -- drop -- m | STOP hazard 0.55 m ahead | Nav2 none (blocked/idle) | robot cmd_vel msgs 0
```

| Field | Meaning |
| --- | --- |
| `odom` / `terrain` | LiDAR odometry and terrain-monitor rates. They should be about 10 Hz |
| `obstacle` / `step` / `drop` | Nearest one in the forward corridor, metres from the body centre (`--` = none) |
| `STOP …` / `clear` | Proximity-stop state and reason |
| `Nav2 …` | Last would-be command; `none (blocked/idle)` while the lock blocks it |
| `robot cmd_vel msgs` | Messages seen on the robot-side `/cmd_vel` in domain 64. **Must stay 0** |

If the scene is not clear at the end of P1, the screen shows `BASELINE NOT
CLEAR`. Press Ctrl+C, clear the area and run the test again.

**Result.** At the end the test prints one line per check. The overall verdicts
are:

| Verdict | Exit code | Meaning |
| --- | --- | --- |
| `PASS` | 0 | Everything checked out |
| `FAIL` | 1 | At least one check failed |
| `ABORTED` | 2 | You stopped it, or a safety guard triggered |
| `INCONCLUSIVE` | 3 | The scene was not clear, so the objects could not be told apart from the background |

The checks:

- P1: surroundings clear, and Nav2 issued commands.
- P2: box detected (compared with the P1 baseline), and no stop.
- P3: stop with reason "ahead".
- **LOCK**: no Nav2 command passed the mux during any stop, and how many were
  blocked.
- P4: low object detected; P5: person detected, and clear at the end.
- LiDAR odometry at least 8 Hz; terrain inputs valid.
- Commands within 0.10 m/s and 0.2 rad/s, with no reverse or lateral motion.
- **Zero robot-side `/cmd_vel` messages.**

### 4.1 Running phases individually

| Command | What it does |
| --- | --- |
| `operator/run_test4.sh --list-phases` | Shows the phases and their instructions |
| `operator/run_test4.sh --phases P3` | Runs only P3, the proximity stop |
| `operator/run_test4.sh --phases P2,P3` | Runs P2 then P3 (always in P1 to P5 order) |
| `operator/run_test4.sh --manual` | Waits for **Enter** before each phase, so you can set up the scene in your own time |
| `operator/run_test4.sh --manual --phases P4` | The options combine |

- **Automatic baseline.** P2, P4 and P5 compare against the empty scene. If you
  run them without P1, a 20 s **BASE** capture runs first: keep the area clear
  during it.
- **P3 needs no baseline.**
- **Manual mode keeps everything running while it waits.** The live line, Nav2
  goals and the safety guards all continue. Each phase starts its 60 s when you
  press Enter, and only those 60 s are scored.
- **Scoring covers what ran.** The result lists only the phases you ran, plus the
  whole-run checks (LiDAR rate, command limits, LOCK, NON-ACTUATING).
  `summary.md` shows "Phases completed".

**Safety guards.** These run every second and abort the test immediately:

- any `/cmd_vel` publisher in domain 8;
- any message on `/cmd_vel` in domain 64;
- `twist_mux` no longer publishing the dry topic.

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
KB on `/livox/points`, both at 10 Hz, about 8.5 MB/s together. Each one arrives
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
