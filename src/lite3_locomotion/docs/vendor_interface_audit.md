# Lite3 movement interface: source audit and deployment gaps

## Verified vendor source

The official [DeepRoboticsLab/Lite3_ROS repository](https://github.com/DeepRoboticsLab/Lite3_ROS)
provides the missing ROS-to-motion bridge. This audit pins the `ros2-foxy` branch
to commit `e4ab5dc1c7f3ff46274162a1ef302253936578aa`. This is source evidence,
not confirmation of the software/firmware deployed on this robot.

The main movement-command source is
[Jetson2Motion.cpp](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/e4ab5dc1c7f3ff46274162a1ef302253936578aa/src/transfer/src/Jetson2Motion.cpp):
`MotionSender::CmdVelCallback` translates body velocity
into three UDP datagrams. It subscribes to both `cmd_vel` and
`cmd_vel_corrected` with best-effort QoS, plus `simple_cmd` and `complex_cmd`.
The callback sends packets directly; the inspected sender has no independent
stale-command timer or shutdown zero-command transmission.

| ROS input | Command code | Payload |
| --- | --- | --- |
| `linear.x` | 320 | Forward velocity |
| `linear.y` | 325 | Lateral velocity |
| `angular.z` | 321 | **Negative** yaw rate |

Each packet contains three signed 32-bit header fields (`cmd_code`, `size=8`,
`type=1`) and a double value, under 4-byte C++ struct packing. Upstream transmits
native struct memory without explicit byte-order conversion. The Python
[preview module](../lite3_locomotion/protocol_preview.py) assumes little-endian
encoding and a 20-byte packet. Actual firmware ABI and version remain unverified.
The module validates and limits commands using the existing policy, returns
bytes only, and contains no socket, transport, startup, or mode-switching code.
It is not a replacement for the complete manufacturer SDK.

The upstream [README](https://github.com/DeepRoboticsLab/Lite3_ROS/blob/e4ab5dc1c7f3ff46274162a1ef302253936578aa/README.md)
documents ROS linear units as m/s and yaw as rad/s: positive x forward, positive
y left, positive ROS yaw left. The sender negates yaw at the wire boundary.
Default target is `192.168.1.120:43893`, receive port `43897`. Defaults do not
prove these addresses belong to the present robot. Do not send these previews.

## SDK distinction

The official [Lite3_MotionSDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK)
is a joint-level SDK (position, velocity, gains, feed-forward torque), not the
body-velocity bridge needed to retain the built-in gait controller. Its reported
one-second joint-command loss protection cannot be assumed to apply to the
separate high-level velocity interface. This audit does not install, build,
start, or configure either vendor package.

## Local deployment evidence

The [pinned source copy](../vendor/Lite3_ROS/SOURCE_MANIFEST.json) is now staged
inside this package for review. It is excluded from ordinary discovery and is
not installed into the active ROS environment. See the
[movement-test review](movement_test_review.md) for compatibility findings and
the offline, non-transmitting bounded adapter prototype.

The current SSH endpoint `192.168.10.78` is present on this environment, along
with `192.168.1.78`. The environment reports `lxc` virtualization and x86_64.
It may be hosted onboard, but this does not establish that it is the motion-host
process environment. `transfer` is not in the active package index. Installed
Lite3 motion-transfer launch links target missing source. The telemetry bridge
is visible in domain 8; its source domain and deployment host remain unknown.

The attached ROS Lab Exercise 7 concerns TurtleBot3 simulation, TF, and a virtual
computer exercise. It does not establish the Lite3 deployment or remote model.
Its example sensor extrinsics must not be treated as calibration for this robot.

## Manual override remains unverified

### New lab-document evidence

The supplied **ROS Lab Exercise 3 – Part II**, page 8, documents:

| `/simple_cmd` code | Documented effect |
| --- | --- |
| `0x21010C02`, `type=0` | Return to Manual mode for handheld-controller commands |
| `0x21010C03`, `type=0` | Enter Navigation/auto mode |

The document states that the handheld controller does not move the robot in
Navigation mode. Thus, stopping autonomous velocity publication alone cannot be
claimed to hand control back to the remote. The future robot adapter must latch
autonomy off, handle stopping, request Manual mode, and establish that the mode
transition actually succeeded. A sent ROS message is not such confirmation.
These codes are document evidence; no mode command has been sent or wired into
the package. Robot firmware compatibility and transition/stop behavior remain
unverified. The current test kill service changes only the local policy state.

The same document says the onboard computer uses ROS 2 Jazzy, whereas this SSH
environment has Humble and reports LXC. This is a further deployment difference
to resolve; it does not establish that the lab document matches every installed
robot software version. The lab's controller initialization and mode procedures
must not be substituted with guesses from the generic Xbox teleop configuration.

The vendor README describes selecting auto/manual modes through its App and an
emergency stop, but does not establish the button sequence for the supplied
physical controller, its firmware, or its priority over SDK commands. The local
university joystick configuration is an Xbox-style `teleop_twist_joy` example;
it cannot identify the supplied remote or prove its override behavior. Joystick
mux priority is not a latched disable of autonomy.

Before a separately approved actuator integration, obtain the robot variant and
controller model, firmware version, manufacturer controller manual, deployed
bridge/source-domain configuration, verified controller watchdog and zero-command
semantics, and documented manual takeover/emergency-stop procedure. This source
audit and offline packet tests do not verify physical stopping or takeover.
