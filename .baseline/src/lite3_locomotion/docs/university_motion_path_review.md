# University Lite3 motion pathway: read-only integration review

Date: 2026-11-12, approximately 12:16–12:23 UTC. Environment: `group8`, ROS 2
Humble inside the onboard LXC. This supplements the
[first-movement review](first_physical_movement_review.md).

## Outcome

The university's documented movement procedure was found. It uses the existing
vendor ROS `transfer` bridge on the robot-side computer, not Nav2 alone and not
an inherently required new Wi-Fi login or server connection. The local university
navigation launcher supplies the upstream velocity/mux portion of that design.

**No currently working, authorized end-to-end motion pathway could be verified
from group8.** Its visible domain-8 bridge exposes telemetry but no outbound
movement input. The installed university workspace contains broken bridge-launch
links, and `transfer` is absent from its sourced package index. A motion bridge
may exist on the LXC host, another authorized domain, or the university server;
the available evidence does not locate or grant access to it.

No network settings, credentials, control modes, services or runtime code were
changed. No motion services/executables were started and no physical command
topics were published. No SSH login, port scan, credential attempt, or new server
connection was made. Only this report was created.

## 1. University lab procedure recovered

Primary local source:
[ROS Lab Exercise 3 – Part II](../../../../.vscode-server/data/agentSessionData/e974975c-548f-40c0-995c-9203534d66b1/attachments/95a78438-bed0-4cef-aeb1-b3203bbbcd85/ROS_Lab_Exercise3%20-%20part_2.pdf),
14 pages, SUTD EPD / 30.119 Intelligent Robotics.

| Pages | Documented practice | Integration implication |
| --- | --- | --- |
| 2 | Two main groups share a dog; only one VC may communicate during autonomous control | Exclusive lab allocation must be established, not inferred from a quiet local topic |
| 4–5, 7 | Manufacturer manual initialization/calibration precedes autonomous use | Starting ROS does not initialize or stand the dog |
| 6–7 | Student VC reaches the robot-side environment on the existing university network; document uses `192.168.10.xx` addressing and ROS domains | Existing lab connectivity is the intended route; no new Wi-Fi credentials are implied |
| 7 | Robot-side bringup is `ros2 launch transfer transfer_launch.py`; its terminal remains running | This is the lab's motion-capable bringup, unlike the current sensor-only launcher |
| 8–9 | `/simple_cmd`, `transfer_interfaces/msg/MotionSimpleCMD`, selects Navigation using `0x21010C03`, and Manual using `0x21010C02`, type 0 | Manufacturer mode is separate from local guard arming; no such command was sent |
| 8 | Normal handheld walking commands do not move the robot in Navigation mode | Handheld joystick activity is not a demonstrated emergency override |
| 9–10 | Student VC runs keyboard teleoperation after robot-side bringup and mode selection | ROS topics carry student commands to the robot-side bridge |
| 11–12 | Goal exercise uses `ros_lite3_pkg`, `robot_goto_goal.py`, and `/leg_odom2`; it discusses control-loop frequency | This is a separate lab goal controller, not proof Nav2 already owns the motion channel |

This is verification of a **documented exercise procedure**, not a historical
execution log proving that a particular student previously moved this exact dog.
The goal script and its `ros_session3_ws` were not found in the accessible
workspaces. No missing credentials from the handout were used or reproduced.

Two ambiguities remain in the handout: it mixes group-ID/robot-ID substitutions
for addresses/domains, and it describes onboard Jazzy whereas this container is
Humble. These may represent host-versus-container layers. They do not justify
changing ROS domains or assuming another group's robot assignment.

[ROS Lab Exercise 7](../../../../.vscode-server/data/agentSessionData/e974975c-548f-40c0-995c-9203534d66b1/attachments/5904c1ce-63ce-4116-8283-058a2ba5bff1/ROS_Lab_Exercise7.pdf)
begins with TurtleBot3 simulation/TF exercises. Its RViz Nav2 goal workflow does
not establish a real Lite3 motor connection.

## 2. Source-verified university command chain

Current university workspace: `/home/user/ros2_ws`.

```text
University Nav2 controller
  -> /cmd_vel_nav
  -> velocity_smoother -> /nav_cmd_vel ------+
                                         |
joy_node -> teleop_twist_joy -> /joy_cmd_vel +-> twist_mux -> /cmd_vel
                                                           |
                               vendor motion_sender subscription
                               inside transfer/jetson2motion
                                                           |
                                  internal UDP -> motion controller
```

The upstream portion is directly supported by available source:

- [lite3.launch.py](../../../../ros2_ws/src/lite3_launcher/launch/lite3.launch.py)
  starts sensor bringup, optional joystick, twist mux, and optional navigation.
  It DOES NOT include a vendor transfer/motion-bridge launch.
- [lite3_bringup.launch.py](../../../../ros2_ws/src/lite3_launcher/launch/include/lite3_bringup.launch.py)
  starts only Fast-LIO, odometry relay, and cloud-to-scan conversion.
- [nav2_bringup.launch.py](../../../../ros2_ws/src/lite3_launcher/launch/include/Navigation/nav2_bringup.launch.py)
  remaps controller output to `cmd_vel_nav`, smoother output to `nav_cmd_vel`,
  and behavior-server output to `nav_cmd_vel` (relative names, root by default).
- [teleop.launch.py](../../../../ros2_ws/src/lite3_launcher/launch/include/Base/teleop.launch.py)
  routes joystick velocity to `joy_cmd_vel`.
- [twist_mux.launch.py](../../../../ros2_ws/src/lite3_launcher/launch/include/Base/twist_mux.launch.py)
  maps `cmd_vel_out` to absolute `/cmd_vel`.
- [twist_mux_topics.yaml](../../../../ros2_ws/src/lite3_launcher/config/twist_mux_topics.yaml)
  assigns navigation priority 10, joystick priority 100, and 0.5 s input timeouts.
  This is a priority policy, not a proven emergency stop or motion-host watchdog.

All these velocity messages are `geometry_msgs/msg/Twist`. With the vendor's
default root namespace, its relative `cmd_vel` subscription is `/cmd_vel`.
Any real deployed namespace/remapping still needs confirmation.

The locally pinned [vendor transfer launch](../vendor/Lite3_ROS/src/transfer/launch/transfer_launch.py)
starts `jetson2motion`, `jetson2app`, and `sensor_checker`. The
[vendor C++ sender](../vendor/Lite3_ROS/src/transfer/src/Jetson2Motion.cpp)
creates a node named `motion_sender`, subscribes to `cmd_vel` and
`cmd_vel_corrected`, and sends forward/lateral/yaw UDP packets. It also accepts
`simple_cmd` and `complex_cmd`. Its source defaults are `192.168.1.120:43893`,
with receive port 43897. These are not observed live parameters.

**The last bridge-to-controller segment is source/document verified only. It is
not currently a runtime-verified connection in group8.** The full transfer
launch has app/service-management side effects; do not start it casually or
create a duplicate bridge merely because the lab lists its name.

## 3. Recovered earlier bridge-launch artifact

The installed `lite3_bringup` prefix exists, but its motion launch and domain
configuration links target a missing `src/lite3_bringup` source directory. In
particular, `motion_transfer.launch.py`, `domain_bridge.launch.py`, and
`config/domain_bridge.yaml` cannot currently be read through those links.

A real compiled launch artifact remains at
[motion_transfer.launch.cpython-310.pyc](../../../../ros2_ws/src/lite3_launcher/launch/__pycache__/motion_transfer.launch.cpython-310.pyc).
It was inspected as data using string extraction and bytecode disassembly;
the module and launch function were NOT executed. It records:

- Package/executable `transfer` / `jetson2motion` only.
- Launch defaults `target_ip=192.168.1.120`, `target_port=43893`,
  `local_port=43897`.
- TF remaps `/tf -> /lite3_tf`, `/tf_static -> /lite3_tf_static`.
- No motion-topic remapping or explicit namespace in that artifact.

This supports an earlier intended local bridge configuration. A cached launch
does not prove successful historical deployment, current firmware compatibility,
permission to operate, or an active server-side bridge. It is not a replacement
for the missing reviewed source/configuration.

## 4. Exact live path, verified separately

After sourcing ROS Humble and the existing university workspace, domain 8 showed:

```text
/lite3_autonomy/controller_server
  -> /lite3_autonomy/nav_cmd_vel [Twist]
  -> /lite3_guarded_adapter
  -> /lite3_locomotion/guarded_test_cmd_vel [Twist]
  -> ZERO discovered subscribers
```

`/cmd_vel`, `/simple_cmd`, `/nav_cmd_vel`, and `/joy_cmd_vel` were not discovered.
There was no visible `motion_sender` or `twist_mux` node. `ros2 pkg prefix transfer`
and `ros2 pkg prefix domain_bridge` both failed even after sourcing the existing
university installation. The `lite3_launcher` and stale `lite3_bringup` prefixes
were discoverable; a package prefix alone does not make its broken launch work.

The active `/lite3_domain_bridge_8` publishes incoming sensor/robot telemetry;
its only reported subscriber is `/parameter_events`. It reports no services.
It does not expose `/cmd_vel` or `/simple_cmd` as outbound inputs. This could be
an intentional lab access boundary or simply the configured topic set; intent
and access policy cannot be inferred from the graph.

No university-server workspace/motion-service configuration is mounted in the
inspected folders, and no existing outbound SSH session to such a server was
identified in this namespace. That is an access/evidence limitation, not proof
that the university has no working bridge elsewhere.

## 5. Compare the three possibilities

| Option | Existing evidence | Required change/prerequisite | Assessment |
| --- | --- | --- | --- |
| 1. Bridge inside group8 LXC | Old local launch artifact; standard internal route; local guarded sender code | Recover approved bridge source/install/configuration; establish reservation, endpoint, domain and stop behavior; avoid duplicate receiver/sender | Technically plausible using existing networking, but not already working here |
| 2. Existing university Ubuntu-server bridge | Visible telemetry bridge proves some existing ROS data sharing, not a server-side motion input | Existing authorized server/host owner must expose its current node/topic/domain/authority information through available lab access; no new connection is assumed | Lowest-change reuse IF an active approved bridge already exists; its existence/access cannot currently be verified |
| 3. Manufacturer Ethernet interface | `eth0` already has `192.168.1.78/24`; route to documented/default `192.168.1.120` is direct, no gateway | Confirm this namespace is attached to the robot's internal Ethernet and identify the actual motion endpoint/firmware | Existing transport candidate, not a separate replacement for a ROS/UDP bridge; no new Wi-Fi credentials inherently required |

Options 1 and 3 can be the SAME architecture: a group8 bridge using the already
configured Ethernet interface. A server bridge may also use Ethernet. They are
not three mutually exclusive motion APIs. No physical cable/link or motion-host
reachability was established from a route entry alone.

**Fewest-change strategy:** reuse the university's existing authorized vendor
bridge wherever it already runs, rather than deploy a second transport. If it
is on the server/host, reuse its approved input after exclusive lab allocation.
If none exists, restore the documented robot-side bridge on the approved host
using existing network configuration. The current evidence cannot rank an
uninspected server as a verified working or fewer-change implementation.

## 6. Integration with lite3_locomotion is not a blind remap

The local autonomy stack and university stack use different topic families:
`/lite3_autonomy/nav_cmd_vel` is not `/nav_cmd_vel` or `/cmd_vel`.

The [guard](../legacy/lite3_legacy/guarded_adapter.py) currently pins robot output
to `/lite3_robot_link/cmd_vel` and requires the new local sender's ready/session
handshake and specific node identities. The legacy vendor bridge has no such
handshake. Publishing to its `/cmd_vel` or simply renaming the vendor node will
not satisfy the current guard. A legacy-bridge integration needs a reviewed
adapter/gateway configuration or implementation; it has not been applied here.

Never remap `/lite3_locomotion/guarded_test_cmd_vel` or the original test topics
to motors. Do not bypass graph checks by spoofing node names. Mux priority is
not a substitute for the guard's latch or an independent emergency stop.

## 7. Remaining prerequisites and exact next evidence

1. **Existing lab pathway:** obtain the host/domain and resolved subscriptions of
   the authorized `motion_sender` (or equivalent), plus its launch source and
   actual `target_ip`, `target_port`, `local_port`, namespace and remappings.
   Include `cmd_vel_corrected`, simple/complex commands, app behavior and other
   groups in the authority check. Use existing authorized access; do not request
   new Wi-Fi credentials merely to discover a ROS input.
2. **Domain bridge:** recover the actual active bridge configuration and owner.
   Determine whether group8 is approved to submit motion or only receive data.
   A command-capable forwarding rule must be explicitly authorized and reviewed;
   do not enable arbitrary bidirectional forwarding or change domains blindly.
3. **Lab allocation:** reserve the shared dog and one command source in accordance
   with the handout. Only the lab/robot owner can confirm current physical command
   authority beyond this container's graph.
4. **Mode and initialization:** match the documented procedure to this actual
   Venture/firmware; confirm calibration, standing, Navigation selection and
   Manual handback. Do not use lack of response to an unverified command as mode
   proof. Any physical initialization/mode transition requires later approval.
5. **Stop readiness:** establish the actual independent emergency control and
   link-loss behavior for Navigation velocities, not just joystick commands.
   The lab's Ctrl+C advice does not prove stopping after process/network failure.
6. **Bounded trial:** retain the earlier proposed +0.03 m/s, 0.5 s one-shot as a
   review candidate, followed by explicit zero and measured/operator-confirmed
   rest. A live pulse runner and validated stop observer remain prerequisites;
   `autonomy_control start` starts exploration, not that bounded trial.

The detailed telemetry and stop-verification gaps are recorded in the
[first-movement review](first_physical_movement_review.md). They are not resolved
merely by locating an existing command subscriber.

**Access conclusion:** this container currently has verified ROS telemetry and
a dry-run command chain, but no verified authorized outbound motion access.
Permission to access the container or receive telemetry must not be treated as
permission to operate another domain/host's motor interface. This does not mean
the robot is unusable; the missing piece is the existing lab bridge/access
configuration and its operator allocation. No movement execution is authorized
by this review.
