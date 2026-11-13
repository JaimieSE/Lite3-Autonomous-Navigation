# Movement-test preparation for review

**Status: bridge startup checked; no physical movement test has been performed.**
The user confirmed `192.168.10.78` is explicitly assigned to this dog. The vendor
motion-host candidate `192.168.1.120` responded to two pings and provided an SSH
banner. No SSH login, mode change, or velocity command was sent. Files and
generated outputs stay inside this package. No existing robot service was changed.
This document records the earlier bridge review. A separate disarmed Nav2
adapter has since been added; see [the autonomy stack](autonomy_stack.md).
It has been tested using isolated dry-run output, not physical movement.
Manual-mode integration is omitted from that stack as requested.

## Validation performed

| Check | Result |
| --- | --- |
| Python package isolated build | PASS |
| Offline unit tests, including bounded sequencing and packaging | 203 passed |
| Opt-in ROS integration cases during this offline run | 11 skipped intentionally; prior isolated run passed 11 |
| Vendor motion, App, and sensor-checker compile/link | PASS, single build job; warnings reviewed below |
| Normal and operator-killed movement-intent previews | Generated locally; confirmations simulated |
| Physical movement and mode switching | Not performed |
| Command-free bridge startup in `/lite3_robot_link` | PASS; zero velocity publishers and one bridge subscriber observed; process stopped |
| Separate shutdown candidate, loopback only | PASS; clean SIGINT exit in 0.164 s, zero outbound command packets |

## Latest connection and shutdown checks

A four-second receive-only probe bound `192.168.1.78:43897` and received no
telemetry from `192.168.1.120`. This does not prove absence of telemetry elsewhere
or establish which destination is configured on the motion host. A packet
capture attempt lacked permission; no privilege escalation was attempted.

The unmodified motion bridge was started for a bounded command-free check in a
dedicated `/lite3_robot_link` namespace, with absolute telemetry topics remapped
away from the existing graph. Its velocity subscription appeared with zero
publishers. No motion or mode commands were published. Normal SIGINT shutdown
blocked in the receiver thread and the bounded probe terminated that process.

The separate [shutdown candidate](../review/bridge_candidate/src/Jetson2Motion.cpp)
retains vendor command behavior and license but uses a 200 ms UDP receive
timeout, skips parsing on timeout, and uses an atomic shutdown flag. The original
pinned source is unchanged. The additional CMake target
`jetson2motion_shutdown_review` compiles this derivative only.

[check_bridge_shutdown.py](../tools/check_bridge_shutdown.py) tests the
derivative with a hardcoded loopback UDP destination, isolated local-only DDS,
and a unique namespace. It initializes no motion publishers. The recorded
[shutdown result](../.colcon/review/shutdown_check.json) shows a clean exit and
no outbound UDP command packets. This fix does not add a physical stop, command
watchdog, Manual-mode acknowledgment, or autonomous navigation capability.
All processes created by these checks have exited.

## Files to review

| File | Role |
| --- | --- |
| [Vendor Jetson2Motion.cpp](../vendor/Lite3_ROS/src/transfer/src/Jetson2Motion.cpp) | Actual official ROS/UDP movement-command source; `MotionSender::CmdVelCallback` |
| [Source manifest](../vendor/Lite3_ROS/SOURCE_MANIFEST.json) | Pinned source commit and per-file SHA256 hashes; downloaded bytes checked against upstream Git blob hashes |
| [bounded_test.py](../lite3_locomotion/bounded_test.py) | Proposed one-shot movement/stop/handback sequencer; returns intents only |
| [Unit tests](../test/test_bounded_test.py) | Preconditions, speed/duration bounds, watchdog, kill, shutdown, no rearming, and missing/late handback confirmation |
| [Dry-run tool](../tools/preview_movement_test.py) | Writes simulated normal and killed runs without importing ROS or opening sockets |
| [Recorded dry run](../.colcon/review/movement_plan.json) | JSON intent trace; all mode confirmations are explicitly simulated |
| [Build wrapper](../review/bridge_build/CMakeLists.txt) | Humble compile/link check without installing or running vendor nodes |
| [Deployment candidate](../review/deployment_candidate.json) | Review-only values with `activation_ready=false`, explicit unknowns, and no automatic start |

The vendor copy contains 15 unmodified files from Lite3_ROS commit
`e4ab5dc1c7f3ff46274162a1ef302253936578aa`, plus a provenance manifest. The MIT
license is preserved. `vendor/COLCON_IGNORE` excludes the copy, including its
duplicate `transfer_interfaces` package, from ordinary colcon discovery.
`review/COLCON_IGNORE` excludes the compatibility wrapper. Nothing is sourced
into the existing robot environment and no system dependencies are installed.

## Proposed first physical test (not executed)

The review plan assumes a supervised forward-only pulse at **0.03 m/s for
0.5 seconds**, nominally 0.015 m of commanded travel. These are placeholder
test values, not validated limits or a guaranteed distance bound. No lateral,
turning, posture, gait, or joint commands are part of the pulse. The offline
sequencer caps speed at 0.05 m/s and duration at one second, regardless of
configuration; those caps also do not establish a safe operating envelope.

The plan starts disarmed. A future actuator implementation must obtain explicit
operator readiness and independently establish Navigation mode before starting
the one-shot sequence. No automatic navigation-mode transition is implemented.
Updates are proposed at 20 Hz. The sequencer latches an abort if the gap reaches
0.2 seconds or its clock regresses. Duration expiry, kill, or shutdown returns:

1. A zero body-velocity intent.
2. A Manual-mode request intent using documented code `0x21010C02`, `type=0`.
3. No further nonzero intents, including after subsequent start attempts.

The intent sequence repeats zero/Manual requests while waiting, and reports a
fault if Manual confirmation is absent at two seconds. This is a proposed
sequence, not proof that repeated Manual requests are accepted or that the
robot has stopped. An actual implementation needs a verified acknowledgment or
operator-confirmation procedure; publishing a mode command alone is insufficient.
No acknowledgment source is implemented or inferred from the generic odometry.
After completion or fault, a new sequence object is required for another trial.

The existing test kill service is unchanged and still controls only the test
node. It is not wired to this offline sequencer or to the robot. The real
kill/handback path, operator trigger, independent watchdog, and mode confirmation
must be implemented and reviewed before this plan can control a robot. A Python
watchdog cannot issue a stop when the process, computer, or network has failed.

## Bridge compatibility and deployment findings

The wrapper explicitly adds `tf2_geometry_msgs` and thread dependencies omitted
from the vendor CMake dependency list, while leaving the vendor source unchanged.
All three review targets (`jetson2motion_review`, `jetson2app_review`, and
`sensor_checker_review`) compiled and linked successfully with one build job.
The sources were compiled only against this x86_64 Humble environment; this does
not validate robot firmware or a Jazzy/ARM deployment. Review binaries reside
under `.colcon/vendor_bridge_build` and are not installed or launched.

The motion sender has no independent command-expiry timer and no zero command in
its destructor. Its UDP success result is not proof of motion-host acceptance.
The motion receiver blocks in `recvfrom`, and its destructor joins the receiver
thread before closing the socket, so stopping the bridge cannot be assumed to
provide prompt or reliable physical stopping. Neither property was tested live.

The vendor launch file starts more than the motion bridge: `Jetson2App.cpp`
can start/stop existing `realsense_ros2.service` and `voa_ros2.service` upon App
messages. A future scoped deployment should not blindly launch that full stack.
Compiler diagnostics also identify obsolete TF headers, incorrect `ssize_t`
printf formats, and `std::string` passed to a `%s` format in the sensor checker.
The latter can cause undefined behavior when that diagnostic path executes.
The unmodified vendor copy is therefore reference/compatibility material, not
an approved production deployment.

## Remaining evidence before activating a transport

The lab document gives the high-level Manual-mode command, but this environment
still lacks a verified deployed bridge/source domain, motion-host address,
firmware match, mode acknowledgment, and independent stop/watchdog behavior.
The address `192.168.1.120:43893` is an upstream default only. The current SSH
endpoint `192.168.10.78` reports x86_64 LXC/Humble; the lab describes onboard
Jazzy. These environments must be reconciled before deployment.

The next approval is for a concrete **deployment-only** configuration on the
verified host, with all command sources accounted for. A physical pulse is a
separate action, performed after the approved stop/handback path is available
and the robot is prepared according to the lab's supervised procedure. This
document deliberately supplies no command that starts the vendor bridge,
enters Navigation mode, stands the robot, or publishes physical velocity.

## Repeat the offline preview

From the package directory:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/preview_movement_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test
```

These commands do not run the vendor binaries or initialize ROS. Existing ROS
integration tests remain opt-in through the separate isolated test runner.
