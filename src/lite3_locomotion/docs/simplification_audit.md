# University navigation reuse: audit and change record

Date: 2026-11-13. Development refactor completed; live action/bridge commissioning
remains unverified. No physical movement, Auto Mode, service startup or university
package changes were performed.

## Corrected environment finding and Git preservation

The earlier restricted tool environment hid university files, ROS and attachments.
After access changed, the onboard environment exposed `/home/user/ros2_ws`, ROS
Humble, both lab PDFs and the supplied architecture snapshot. The earlier report
of missing files applied to that tool snapshot, not to the onboard computer.

The workspace's `.git` directory actually contained no repository. Before changing
runtime code, initialized a local repository and committed the complete project
source, tests, configuration, vendor/review/recovery references and previous
audit at `e2e0552`. Generated build/log output and unrelated university/driver
packages were excluded. Branch `preserve/pre-university-reuse-2026-11-13` preserves
that baseline. Refactoring is on `refactor/university-navigation`.

## Actual university evidence inspected

| Evidence | Findings |
| --- | --- |
| Lab 4 PDF, mapping/navigation exercises | The installed `lite3_launcher` provides SLAM and navigation; mapped destinations are selected in RViz. Auto Mode and system-driver operations are separate procedures. None were executed. |
| Lab 5 PDF, drain patrol | Adds an application using manually recorded waypoints and object detection; this does not supply autonomous frontier selection. |
| Supplied `file_autonomy_stack.md` snapshot, including line 118 | Describes the earlier project stack and verification workflow. It is not proof of current university runtime interfaces. Attachment left unchanged. |
| [Top-level launcher](../../../../ros2_ws/src/lite3_launcher/launch/lite3.launch.py) | Includes base, optional joystick, mux and optional navigation; no motion-transfer launch. Defaults: navigation and joystick enabled, SLAM disabled. |
| [Base launch](../../../../ros2_ws/src/lite3_launcher/launch/include/lite3_bringup.launch.py) | Fast-LIO, odometry relay, cloud-to-scan. No project replacement needed. |
| [Navigation launch](../../../../ros2_ws/src/lite3_launcher/launch/include/lite3_navigation.launch.py) | Existing composed Nav2; SLAM toolbox for mapping, AMCL/map server otherwise; empty namespace by default. |
| [Nav2 bringup](../../../../ros2_ws/src/lite3_launcher/launch/include/Navigation/nav2_bringup.launch.py) | Planner/controller/BT navigator, smoother, behaviors, waypoint follower and lifecycle manager; both composed and standalone variants inspected. |
| [University parameters](../../../../ros2_ws/src/lite3_launcher/config/nav2_params.yaml) | Navfn planner, MPPI controller, `map`/`odom`/`base_link`, `/scan`, `/odom`; unknown-space planning enabled. Older BT/recovery parameter names also appear: do not infer successful runtime bringup from YAML alone. |
| [Mux settings](../../../../ros2_ws/src/lite3_launcher/config/twist_mux_topics.yaml) and launch | Navigation priority 10, joystick 100, input expiry 0.5 s; output `/cmd_vel`. Expiry is not proof of physical stopping. |
| [Scan settings](../../../../ros2_ws/src/lite3_launcher/config/pointcloud_to_scan.yaml) | Full-circle scan transformed to `base_link`; matches coordinator's inspected input convention. |
| [Waypoint navigator](../../../../ros2_ws/src/waypoint_navigator/waypoint_navigator/waypoint_navigator.py) | Uses the installed NavigateToPose action for supplied poses. Its constructor references bare `action_name` after assigning `self.action_name`; this source issue was noted and left untouched. Our coordinator does not depend on this client. |
| [Drain patrol](../../../../ros2_ws/src/drain_patrol/drain_patrol/waypoint_follower.py) | A separate line-following waypoint controller that directly publishes velocities and a sit command on detection. Do not start it alongside exploration as an independent command source. |
| Installed action definition | `nav2_msgs/action/NavigateToPose`: PoseStamped + optional behavior-tree path; result/status/cancellation handled by rclpy. The new client uses this installed definition. |
| Installed launcher | Share-directory launch symlinks resolve to the inspected university source. Source provenance is captured by [SHA-256 manifest](university_source_manifest.sha256); university source has no accessible Git repository. |

Read-only domain-8 discovery showed `/lite3_domain_bridge_8` publishing telemetry,
including LiDAR, IMU and odometry. It exposes no action servers and no movement
subscriptions: only `/parameter_events` was listed as a subscriber. No Nav2,
`/map`, `/scan` or outbound `/cmd_vel` was discovered. No missing services were
started to obtain evidence. The installed `transfer` package is not discoverable
here; stale `lite3_bringup` motion/domain-bridge launch links point to absent
source. This does not establish what runs on the separate system computer.

Default source routing, verified from both Nav2 launch variants:

```text
Existing frontier client -> /navigate_to_pose
  -> university BT navigator -> Navfn / MPPI
  -> /cmd_vel_nav -> velocity_smoother -> /nav_cmd_vel
  -> twist_mux (also /joy_cmd_vel) -> /cmd_vel
  -> existing system-computer bridge (deployment not yet inspected)
```

The pinned vendor reference subscribes to `cmd_vel`, `cmd_vel_corrected`, simple
and complex commands; its transport defaults are not evidence of the actual
system host. Its source provides no equivalent of the project's session
handshake or independent expiry policy. No UDP packet was sent to a robot.

## Changes, retained functionality and retirement

| Component | Decision and reason |
| --- | --- |
| Frontier search/ranking | Retained unchanged: bounded search and autonomous free-side goals are the project's new functionality. |
| Mission coordinator | Adapted to existing root NavigateToPose/lifecycle interfaces with explicit namespace support; removed dependency on the project's guard heartbeat, sender identities and transport handshake. |
| Mission safety | Retained goal budget/deadline, radius, map/TF freshness, failure cooldown and late-accept cancellation. Consolidated scan validity/clearance/header freshness and session/pose bounds into the coordinator. |
| Cancellation | Added unresolved-goal tracking, terminal-result confirmation and a 3 s cancellation deadline; rejected/unconfirmed cancellation blocks restart. Late acceptance after stop is canceled. Shutdown attempts bounded cancellation while ROS remains valid. This is navigation supervision, not physical stop confirmation. |
| Default launch | Six ROS nodes reduced to one project coordinator. No Nav2 servers/lifecycle manager, velocity gate, driver, transport or mode service. It accepts dry-run only; no action client is created in preview. |
| Mission CLI | Starts/stops only mission services. It no longer arms any adapter or sender. |
| Duplicate Nav2 planner/controller | They were standard Navfn and Regulated Pure Pursuit plugins, not new custom algorithms. Retired their launch/configuration from active installation after replacement contract tests passed. Original files are in [legacy](../legacy/README.md) and Git. |
| Custom UDP sender/commissioning executable | Removed from installed console entry points; no replacement sender was written. Retained source and safety/protocol unit tests because the real bridge watchdog remains unverified. |
| Guard adapter and policies | Retained source and deterministic tests; adapter no longer installed as an executable or launched. Velocity clamping/watchdog policies are not in the university velocity path. Physical deployment must verify equivalent enforcement before assuming safety. |
| Test controller / marker preview | Retained optional non-actuating tools; neither is part of the default mission runtime. |
| Old integration tests/configuration | Archived the duplicated-stack Nav2/sender end-to-end fixtures and sender runner under `legacy`, excluded from installation and ordinary test collection. Historical tests require the original branch; they are not falsely presented as passing against the new architecture. |
| Dependencies | Removed direct Nav2 server/plugin and YAML dependencies. Kept message/action/lifecycle and launch dependencies required by the client. |
| Documentation | Rewrote README and current architecture guide; replaced duplicate root architecture document with a link. Earlier robot commissioning documents are marked historical. |
| Build | Fresh `.colcon/university_build` and `.colcon/university_install` avoid exposing stale sender scripts/configs from the earlier `.colcon/install`. The old installation is retained, not silently reused or deleted. |

No university package was edited. Source files were checksummed before and after
the work. No implementation was discarded: retired configuration/tests are
archived, all original code is preserved by the baseline branch, and the legacy
transport/safety modules remain available for policy regression tests.

## Verification

Baseline before refactor: **282 passed, 23 opt-in cases skipped**.
The first replacement action-contract suite passed **17/17** before archiving
any duplicate configuration or integration fixtures. An additional installed
launch test verifies exactly one preview coordinator and no command topics.
Final validation: **291 unit tests passed, 30 opt-in tests skipped** in the
ordinary suite; **18 isolated action/launch integration tests passed** in
19.49 s. The fresh university-overlay colcon build passed. Editor diagnostics
and `git diff --check` reported no errors. All files in the university-source
checksum manifest remained unchanged. The fresh installation exposes only
coordinator, mission CLI, marker preview and the non-actuating test controller;
no sender, guard adapter or pulse executable is installed.

The runner enforces Fast DDS, localhost-only UDP transport, no shared memory,
no discovery-server overrides, and domains 40–89. Each test context checks an
empty graph before starting its mock. Separate test domains avoid delayed DDS
teardown from previous contexts. Test sensors and mock lifecycle/action servers
never publish velocity, posture or mode commands. The suite covers:

- Idle startup, no-dispatch preview even with a server available, and installed
  launch containing only the preview coordinator.
- Root and namespaced action/lifecycle bindings, goal frame/quaternion/default
  BT, rejection/abort/success, cooldown, budget and deadlines.
- Explicit mission CLI with no arm service; sensor/header/map/TF health and
  lifecycle loss; session and radius stops.
- Cancellation acknowledgement versus terminal result, rejected cancellation,
  no restart while unresolved, and late acceptance after stop.

A mock validates the client contract; it does not verify university Nav2 bringup,
planning quality, recovery behavior or bridge/firmware behavior. Those remain
explicit deployment checks, and no claim of physical testing is made.

## Remaining terrain work

See [terrain traversal](autonomy_stack.md#terrain-traversal): calibrated terrain
perception, slope/roughness/step/drop-off assessment, traversability costs,
uncertainty-aware route/speed policies and any necessary verified gait interface.
Retain university navigation and vendor locomotion as the execution foundation.
Before live exploration, inspect the existing system-computer bridge, command
sources, independent watchdog/stop behavior and current action ownership using
read-only access. No service start, Auto Mode or physical trial is part of this
refactor.
