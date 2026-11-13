# Build-only recovery candidate: transfer/jetson2motion

**NOT APPROVED FOR EXECUTION OR DEPLOYMENT. Do not run this binary.**
It retains the original vendor physical IP default and accepts commands without
arming. The review YAML is a record, NOT an enforced safety interlock. An existing
publisher can trigger transmission immediately after startup. No launch file is
provided or installed. Building/installing into the isolated prefix does not
start a ROS node, open the robot transport, change modes or send robot packets.

## Scope and provenance

This is a packaging/build wrapper for the existing official Lite3_ROS source at
commit `e4ab5dc1c7f3ff46274162a1ef302253936578aa`. It references
[Jetson2Motion.cpp](../../vendor/Lite3_ROS/src/transfer/src/Jetson2Motion.cpp) and
[protocol.hpp](../../vendor/Lite3_ROS/src/transfer/include/protocol.hpp) unchanged,
and verifies their pinned SHA256 values before configuring. See the
[source manifest](../../vendor/Lite3_ROS/SOURCE_MANIFEST.json).

The university's recovered historical wrapper requested package `transfer`,
executable `jetson2motion`, with `target_ip`, `target_port`, and `local_port`,
defaults `192.168.1.120`, `43893`, `43897`, plus `/tf -> /lite3_tf` and
`/tf_static -> /lite3_tf_static`. These values are historical, not verified for
the physical dog. The earlier wrapper source and domain-bridge YAML are missing;
this candidate does not invent replacements or claim university approval.

The full [vendor launch](../../vendor/Lite3_ROS/src/transfer/launch/transfer_launch.py)
also starts `jetson2app` and `sensor_checker`. This candidate deliberately builds
and installs only the historical motion executable. It still creates BOTH vendor
nodes, `motion_sender` and `motion_receiver`; it is not a sender-only binary.

| Detail | Historical university wrapper | Official vendor launch | Recovery candidate |
| --- | --- | --- | --- |
| Driver | `transfer/jetson2motion` | Same driver, plus app/checker | Same motion executable only |
| Motion parameters | `target_ip`, `target_port`, `local_port` | Same three, same historical defaults | Original source retains defaults; no launch/config assigns a physical target |
| TF | Remapped to `/lite3_tf`, `/lite3_tf_static` | No such launch remaps | No launch installed; deployment remaps unresolved |
| Source behavior | Original wrapper source missing; cached configuration recovered | Original vendor C++ | Same pinned C++ and protocol header, no runtime patches |
| Approval | Lab workflow documented; current deployment unknown | Public source is not lab deployment approval | Build-only, explicitly unapproved |

For missing source-package paths and the existing university command mux, see the
[university recovery review](../../docs/university_motion_path_review.md). The
approved `lite3_bringup` wrapper source/package metadata, active domain-bridge
launch/YAML and actual host/domain mapping still need recovery from the lab's
approved installation or backup. Rebuilding this motion target does not restore
those files or provide permission to submit movement.

## Humble compatibility and dependencies

The untouched vendor CMake configures with the installed dependencies but its
motion target fails to compile: `tf2_geometry_msgs/tf2_geometry_msgs.h` is not
found because `tf2_geometry_msgs` is not declared/exported to that target.

This wrapper supplies explicit dependencies on `ament_cmake`, `rclcpp`,
`std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`, `tf2_msgs`, `tf2`,
`tf2_ros`, `tf2_geometry_msgs`, and existing `transfer_interfaces`, plus system
`Threads::Threads`. They are already available in this x86_64 Humble environment;
no dependency installation is needed. C++14 is retained. The wrapper package
manifest declares the libraries used, rather than copying upstream's incomplete
manifest and inappropriate interface-package group membership.

The existing installed MotionSimpleCMD/MotionComplexCMD message definitions were
compared with the pinned vendor files and match. Do not build the duplicate
vendor `transfer_interfaces` package or overwrite the installed one. The
containing recovery directory has COLCON_IGNORE so ordinary workspace discovery
does not introduce another `transfer` package.

Warnings in original source include deprecated TF forwarding headers, anonymous
struct extensions, unused send-result variables, and printf format mismatches
for `ssize_t`. The full-vendor app/checker targets are
not part of this build; prior compatibility review identified a separate unsafe
string-format diagnostic in the checker. Build fixes here do not correct vendor
runtime defects, validate firmware ABI, or establish physical operation.

## Isolated build plan

All candidate output stays inside the locomotion package, outside the university
workspace `/home/user/ros2_ws` and its installed prefixes. The wrapper remains in
the same checkout as its pinned vendor tree; it is not a standalone source copy.

Run only these BUILD commands in a dedicated child/build shell:

```bash
source /opt/ros/humble/setup.bash
source /home/user/lite3_ws/install/transfer_interfaces/share/transfer_interfaces/local_setup.bash

cmake \
  -S /home/user/lite3_ws/src/lite3_locomotion/recovery/transfer \
  -B /home/user/lite3_ws/src/lite3_locomotion/.colcon/recovery_transfer/build \
  -DCMAKE_INSTALL_PREFIX=/home/user/lite3_ws/src/lite3_locomotion/.colcon/recovery_transfer/install/transfer \
  -DBUILD_TESTING=OFF
cmake --build /home/user/lite3_ws/src/lite3_locomotion/.colcon/recovery_transfer/build \
  --target jetson2motion --parallel 1
cmake --install /home/user/lite3_ws/src/lite3_locomotion/.colcon/recovery_transfer/build
```

No command above executes `jetson2motion`. Do NOT source the candidate prefix
into your normal/live shell. Do NOT remove COLCON_IGNORE, globally add this
prefix, replace university symlinks, install system packages or run rosdep.

Safe build verification consists of compiler/linker output, install-manifest
inspection, ELF dependency inspection, and read-only package-index discovery
with AMENT_PREFIX_PATH set only for that inspection process. `ros2 pkg
executables transfer` lists the installed candidate without starting it. Do not
use `ros2 run` or `ros2 launch` as a package-discovery check.

The build/install outcome and exact inspected prefix are recorded in
[validation.md](validation.md) after verification. No bridge runtime check is
authorized by this plan. An isolated compile does not validate ARM/Jazzy or the
dog's firmware; approval of this packaging wrapper is separate from deployment.

## Runtime issues that remain

- This is the legacy high-level UDP body-velocity interface, not the separate
  joint-level MotionSDK. Codes 320/325/321 carry forward/lateral/negative ROS yaw.
- Both `cmd_vel` and `cmd_vel_corrected` can send motion. `simple_cmd` and
  `complex_cmd` can send arbitrary vendor commands. All are relative names, so
  resolved namespace/remappings must be reviewed together.
- No finite-value validation, velocity limit, arming, command timeout, zero-on-
  shutdown, physical mode confirmation or stop acknowledgment exists in the
  original sender. UDP send success is not robot acceptance.
- The receiver binds INADDR_ANY on its local UDP port; check for an existing
  receiver before deployment. It blocks on receive and joins before socket close,
  with a non-atomic shutdown flag. Clean bounded Ctrl+C exit is not established.
- Vendor telemetry uses a steady-clock timestamp, leaves some odometry frames
  empty and does not establish source authentication. It may conflict with the
  existing sensor/domain bridge. A namespace does NOT isolate absolute topics:
  `/imu/data`, `/handle_state`, `/us_publisher/ultrasound_distance` and TF require
  explicit reviewed remapping. The historical two TF remaps alone are insufficient
  to isolate every telemetry publication.
- No independent motion-host watchdog is introduced. Sender, host or network
  loss can prevent a stop. Do not infer the Navigation interface's loss behavior
  from the joystick-axis timeout or joint-SDK protection.

The separately reviewed shutdown derivative is NOT silently substituted into
this candidate. Any such behavior change needs its own explicit review.

## Eventual connection to the guarded controller (not implemented here)

The intended approved data path remains:

```text
Nav2 -> guarded adapter -> approved vendor cmd_vel subscription
  -> transfer/jetson2motion -> verified motion-host endpoint
```

The present [guarded adapter](../../lite3_locomotion/guarded_adapter.py) is NOT
drop-in compatible with this original vendor executable. It pins physical output
to `/lite3_robot_link/cmd_vel` and expects the previously created local sender's
ready/session-token handshake. The vendor executable has no such handshake.
Renaming a node or remapping the dry-run topic must not be used to bypass this.

Before deployment, explicitly choose and review a vendor-transport integration
profile for the existing guard: preserve finite/speed/scan/TF/session checks and
explicit operator arm; validate the real vendor subscriber's approved resolved
topic, process configuration and all competing inputs; replace the local-sender-
specific readiness assumption with evidence appropriate to the original bridge.
Any external independent watchdog/lease requires a separately validated design;
ROS discovery and process liveness alone do not prove a stop path or robot mode.
No new transport implementation is required, but this compatibility work is not
performed by the present build wrapper.

Without that reviewed guard integration, do not use the current robot-output
launch or `autonomy_control start` to test this candidate. The latter starts
frontier exploration, not a short forward-only commissioning pulse. Keep the
original test topics non-actuating.

## Deployment and safety approval gates

1. Lab owner approves this exact candidate/provenance and restores or supplies
   the missing approved motion wrapper/domain-bridge configuration. Do not treat
   the recovery prefix or review YAML as university authorization.
2. Identify the sole motion operator, allocated dog, approved host/domain,
   namespace, all four command inputs, actual motion endpoint and firmware match.
   Do not duplicate a bridge already operating on a university host/server.
3. Confirm installed receiver/telemetry ownership, port availability and clock/
   frame semantics; account for absolute publications and data already bridged
   into group8. No new Wi-Fi credentials or server connection is presumed.
4. Match manufacturer initialization, standing, Navigation authorization, mode
   observation and Manual handback to this Venture. No mode changes occur here.
5. Establish independent emergency stopping and exact high-level command-loss
   behavior. Resolve the original receiver shutdown issue before relying on
   service shutdown for lifecycle management; never call Ctrl+C an emergency stop.
6. Review the guard's legacy-bridge integration, a genuinely bounded forward-only
   trial, zero sequence, fresh calibrated odometry and operator stop observation.
   A receipt acknowledgment cannot substitute for a verified physical stop.
7. Obtain separate deployment/startup approval, then separate explicit supervised
   physical-test approval with a safety observer. Building is not permission to
   start the bridge, switch modes, stand/sit or publish zero/nonzero commands.

Unresolved values are listed in [deployment_review.yaml](deployment_review.yaml).
They must be resolved with evidence before physical deployment, not replaced by
historical defaults or guessed values.
