# Complete package audit and file disposition

Prepared before further relocation/removal on 2026-11-13. Baseline: commit `11e4776`, branch `preserve/pre-code-audit-2026-11-13`.

**Permanent delete list: empty.** Functioning legacy code will be relocated outside the installable package, with offline regression tests retained. No shared service or university file will be modified. This is the pre-change disposition list, followed by a final verification record.

## Decisions before edits

- Keep frontier logic, coordinator, marker preview, mission CLI, non-actuating controller and command-policy tests.
- Modify the packaging boundary, coordinator scan import, metadata tests, integration tests, test runner and current docs.
- Archive six retired motion/transport modules and five associated unit-test files. Retain the bounded simulation script under legacy tools.
- Keep pinned vendor source, review/recovery candidates and licenses as excluded reference material; do not build or deploy them.
- Delete nothing permanently. Generated build/install/log directories are excluded from source inventory; old artifacts are retained, with fresh output under `.colcon/audit_build` and `.colcon/audit_install`.

## Every source/reference file

Paths below identify the baseline file. Archive destinations are the resulting retained paths.

| Baseline file | Decision | Reason / retained destination |
| --- | --- | --- |
| `README.md` | Modify | Describe active/archived boundary, test commands and bridge reuse. |
| `autonomy_stack.md` | Keep | Architecture, provenance or historical evidence; historical command paths need relocation notes. |
| `docs/autonomy_stack.md` | Modify | Update safety-code ownership and evidence about the existing system bridge. |
| `docs/commissioning_pulse.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `docs/daily_startup_checklist.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `docs/exploration_preview.md` | Keep | Architecture, provenance or historical evidence; historical command paths need relocation notes. |
| `docs/first_physical_movement_review.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `docs/movement_test_review.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `docs/robot_output.md` | Keep | Architecture, provenance or historical evidence; historical command paths need relocation notes. |
| `docs/simplification_audit.md` | Modify | Link the complete file inventory and final validation results. |
| `docs/university_motion_path_review.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `docs/university_source_manifest.sha256` | Keep | Architecture, provenance or historical evidence; historical command paths need relocation notes. |
| `docs/vendor_interface_audit.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `docs/walking_trial_2026-11-13.md` | Modify links only | Preserve historical evidence and update source links to retained archive destinations. |
| `launch/autonomy.launch.py` | Keep | ROS package metadata, resource or launch infrastructure. |
| `legacy/COLCON_IGNORE` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/README.md` | Modify | Document how retained offline regression tests and simulation can still run. |
| `legacy/config/nav2_frontiers.yaml` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/config/navigate_frontiers.xml` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/config/navigate_through_frontiers.xml` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/launch/autonomy.launch.py` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/test/test_autonomy_integration.py` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/test/test_motion_sender_integration.py` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/test/test_robot_output_loopback.py` | Keep excluded | Historical stack fixture; not installed or started. |
| `legacy/tools/run_motion_sender_integration.py` | Keep excluded | Historical stack fixture; not installed or started. |
| `lite3_locomotion/__init__.py` | Keep | Active exploration or optional non-actuating command-policy tool. |
| `lite3_locomotion/autonomy_control.py` | Keep | Active exploration or optional non-actuating command-policy tool. |
| `lite3_locomotion/bounded_test.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/lite3_legacy/bounded_test.py](../legacy/lite3_legacy/bounded_test.py). Active imports must be removed first. |
| `lite3_locomotion/command_policy.py` | Keep | Active exploration or optional non-actuating command-policy tool. |
| `lite3_locomotion/commissioning_pulse.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/lite3_legacy/commissioning_pulse.py](../legacy/lite3_legacy/commissioning_pulse.py). Active imports must be removed first. |
| `lite3_locomotion/controller_node.py` | Keep | Active exploration or optional non-actuating command-policy tool. |
| `lite3_locomotion/exploration_coordinator.py` | Modify | Import the retained scan check from active safety; also correct result-subscription exceptions and immediately completed goal callbacks, with regression tests. |
| `lite3_locomotion/exploration_preview.py` | Keep | Active exploration or optional non-actuating command-policy tool. |
| `lite3_locomotion/frontiers.py` | Keep | Active exploration or optional non-actuating command-policy tool. |
| `lite3_locomotion/guarded_adapter.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/lite3_legacy/guarded_adapter.py](../legacy/lite3_legacy/guarded_adapter.py). Active imports must be removed first. |
| `lite3_locomotion/motion_sender.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/lite3_legacy/motion_sender.py](../legacy/lite3_legacy/motion_sender.py). Active imports must be removed first. |
| `lite3_locomotion/movement_guard.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/lite3_legacy/movement_guard.py](../legacy/lite3_legacy/movement_guard.py). Active imports must be removed first. |
| `lite3_locomotion/protocol_preview.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/lite3_legacy/protocol_preview.py](../legacy/lite3_legacy/protocol_preview.py). Active imports must be removed first. |
| `package.xml` | Modify | Remove unused launch_ros runtime dependency; declare actual planner dependencies as test-only. |
| `recovery/COLCON_IGNORE` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `recovery/transfer/CMakeLists.txt` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `recovery/transfer/README.md` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `recovery/transfer/deployment_review.yaml` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `recovery/transfer/package.xml` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `recovery/transfer/validation.md` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `resource/lite3_locomotion` | Keep | ROS package metadata, resource or launch infrastructure. |
| `review/COLCON_IGNORE` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `review/bridge_build/CMakeLists.txt` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `review/bridge_candidate/LICENSE` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `review/bridge_candidate/include/protocol.hpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `review/bridge_candidate/src/Jetson2Motion.cpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `review/deployment_candidate.json` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `setup.cfg` | Keep | ROS package metadata, resource or launch infrastructure. |
| `setup.py` | Modify | Explicitly install only the active package and current documentation; prevent legacy code from shipping. |
| `test/loopback_dds.xml` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_bounded_test.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/test/test_bounded_test.py](../legacy/test/test_bounded_test.py). Active imports must be removed first. |
| `test/test_commissioning_pulse.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/test/test_commissioning_pulse.py](../legacy/test/test_commissioning_pulse.py). Active imports must be removed first. |
| `test/test_external_navigation.py` | Modify | Add an isolated real Navfn planning check using university configuration; retain mock/dry-run tests. |
| `test/test_external_options.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_frontiers.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_motion_sender.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/test/test_motion_sender.py](../legacy/test/test_motion_sender.py). Active imports must be removed first. |
| `test/test_movement_guard.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/test/test_movement_guard.py](../legacy/test/test_movement_guard.py). Active imports must be removed first. |
| `test/test_package_metadata.py` | Modify | Enforce exact active module/entry-point boundary and absence of transport. |
| `test/test_protocol_preview.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/test/test_protocol_preview.py](../legacy/test/test_protocol_preview.py). Active imports must be removed first. |
| `test/test_ros_integration.py` | Modify | Keep all tests; point their installed-package assertions at the fresh audited installation. |
| `test/test_stop_states.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_timeout.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_topic_isolation.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_validation.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `test/test_velocity_limits.py` | Keep | Frontier, policy, package, isolation or integration regression coverage. |
| `tools/check_bridge_shutdown.py` | Keep | Isolated verification or pinned-source provenance tooling; no default production node. |
| `tools/fetch_vendor_source.py` | Keep | Isolated verification or pinned-source provenance tooling; no default production node. |
| `tools/preview_movement_test.py` | Archive | Obsolete standalone motion stack or its fixtures; retain at [legacy/tools/preview_movement_test.py](../legacy/tools/preview_movement_test.py). Active imports must be removed first. |
| `tools/run_autonomy_integration.py` | Keep | Isolated verification or pinned-source provenance tooling; no default production node. |
| `tools/run_ros_integration.py` | Modify | Use the current fresh installation instead of stale pre-refactor executables. |
| `vendor/COLCON_IGNORE` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/LICENSE` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/README.md` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/README_ZH.md` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/SOURCE_MANIFEST.json` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/CMakeLists.txt` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/include/protocol.hpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/launch/transfer_launch.py` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/package.xml` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/src/Jetson2App.cpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/src/Jetson2Motion.cpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/src/SensorChecker.cpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer/src/SensorsLogger.hpp` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer_interfaces/CMakeLists.txt` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer_interfaces/msg/MotionComplexCMD.msg` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer_interfaces/msg/MotionSimpleCMD.msg` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |
| `vendor/Lite3_ROS/src/transfer_interfaces/package.xml` | Keep excluded | Pinned vendor/deployment evidence, interface definition or license; not an active runtime dependency. |

Baseline inventory: **92 source/reference files**. Generated roots `.colcon/` and `log/` contain old build/install caches and historical logs, not extra production packages. Workspace sibling `transfer_interfaces` and `livox_ros_driver2` are outside this package and remain unchanged.

## Architecture / dependency audit

University Lab 4 establishes the installed launcher for mapping and autonomous navigation. Lab 5 describes manually chosen drain-patrol waypoints, which do not provide frontier exploration. The attached architecture pointer is our project documentation, not an independent university interface definition. The actual university source and installed action definition were inspected; their pinned hash manifest is retained.

The existing university motion path is Nav2 -> cmd_vel_nav -> velocity smoother -> nav_cmd_vel -> twist_mux -> /cmd_vel -> transfer motion_sender -> vendor controller. The system computer uses domain 64; group 8 reaches it through the existing bridge. The project should submit NavigateToPose goals, never implement a second UDP transport. Direct motion/posture commands are not part of the current project launch.

Active modules currently reference the legacy movement_guard module only for scan_is_clear. Extracting that pure predicate allows all six legacy modules to leave the installed package without dropping the useful scan check. Legacy dependencies form a separate graph: guarded_adapter -> movement_guard -> active command_policy; motion_sender -> protocol_preview -> active command_policy; bounded_test -> active command_policy; commissioning_pulse is standalone ROS transport tooling. Their unit tests and offline trace generator must follow the relocated namespace.

## Nodes and interfaces

| Component | Role / activation | Interfaces |
| --- | --- | --- |
| exploration_coordinator | Only default launched node; idle, preview-only | /map, /scan, TF; read-only Nav2 lifecycle; mission Trigger services; JSON status; optional existing NavigateToPose client |
| exploration_preview | Optional markers only | /map, TF -> exploration_markers |
| autonomy_control | Short-lived mission CLI | start_exploration / stop_exploration; no mode or arm service |
| locomotion_controller | Optional non-actuating test harness | Fixed test velocity topics and test kill/reset services only |
| Legacy guarded adapter / sender / pulse | Excluded standalone stack or commissioning experiments | Old guarded-test topics, custom sender handshake, UDP, or direct cmd_vel; no active dependency |
| University nodes | Shared external prerequisites, never started/restarted by this audit | SLAM/localization, TF, Nav2 servers, smoother/mux, existing transfer bridge |

No custom .msg, .srv or .action is needed by the active package. It consumes standard ROS messages/services and nav2_msgs actions. Pinned transfer interfaces remain reference material; the workspace sibling interfaces belong to the existing bridge infrastructure and are not to be removed.

## Dependencies

Runtime: rclpy, geometry_msgs, nav_msgs, sensor_msgs, std_msgs, std_srvs, action_msgs, lifecycle_msgs, nav2_msgs, tf2_ros_py, visualization_msgs, launch and ament_index_python. launch_ros is unused by the current ExecuteProcess launch and can leave runtime dependencies. Nav2 servers/plugins remain external runtime prerequisites rather than packages started by this project. nav2_planner, nav2_navfn_planner and YAML parsing are needed only for an isolated real-planning regression test.

## Preservation and validation plan

1. Preserve the exact baseline in Git before any relocation (done).
2. Publish this keep/modify/archive/delete list before touching implementation (this document).
3. Extract scan safety; remove all active dependencies on archived modules; move source and tests without permanent deletion.
4. Verify active unit suite, retained offline legacy policies, installed-package contents, real-DDS mock navigation/dry-run, actual university Navfn planning in an isolated test domain and the non-actuating controller integration.
5. Recheck university source hashes. No physical commands, mode changes, shared service transitions or second motion bridge are permitted.

## Final verification

Implemented without permanently deleting source. Six legacy motion modules and five unit-test files were relocated; the bounded simulation tool moved with them. Added [safety.py](../lite3_locomotion/safety.py), [scan tests](../test/test_safety.py), the legacy namespace marker and this audit. The installed Python package now contains **8 modules instead of 13**, with only the coordinator required by the default launch. No active module imports legacy code, sockets or transfer interfaces; metadata tests enforce this boundary.

Two coordinator defects found during the code audit were fixed: exceptions while attaching a result future now request cancellation and latch restart disabled; state is established before callbacks are registered, so an immediately ready result cannot restore a completed goal handle. Both have focused integration coverage. Mission behavior, bounds and existing safety checks otherwise remain intact.

| Verification | Final result |
| --- | --- |
| Fresh isolated colcon build | Passed; only the selected project package built |
| Active unit suite | **153 passed, 32 opt-in cases skipped** |
| Retained legacy offline policies/protocols | **153 passed, 1 opt-in timing test skipped** |
| Navigation, preview launch and real university planner | **21 passed**, including actual Navfn planning on a synthetic map, mock NavigateToPose contract, cancellation/error paths and zero command topics |
| Fixed-topic non-actuating controller integration | **11 passed**, including limiting, expiry, invalid input, kill/reset and remapping rejection |
| Offline bounded simulation | Passed; generated intent traces only |
| University source manifest | All **42** source/reference hashes unchanged |
| Shared-service changes / extra motion bridge / physical commands | **None during this audit** |

The active and legacy test counts are reported separately because scan-policy regression is intentionally retained in both suites. Lower active counts reflect relocation, not silently discarded tests. Real-DDS suites force loopback transport and domains 40–89; no live domain is used for goal submission, planner lifecycle transitions or test commands. The real-planner check starts a test-only namespaced planner and remaps its map/TF to synthetic inputs. It starts no controller or motion transport. The optional test-controller suite publishes only its fixed non-actuating topics.

Initial verification exposed two test-wiring issues, both corrected: old hard-coded install paths in the controller harness, and the namespaced planner's relative map topic. The initial failed planner attempt required cleanup of its own unresponsive process; the test now owns and cleans up its planner process group. Final suites passed and no test processes remain. No shared processes were targeted.

Current bridge evidence: read-only system-domain-64 discovery confirms one `/motion_sender` with Twist inputs `/cmd_vel` and `/cmd_vel_corrected`, plus `/simple_cmd` and `/complex_cmd`; live target parameters are `192.168.1.120:43893`. Lab 4's prose mentions `/simple_command`, whereas actual source/runtime use `/simple_cmd`; the current project needs neither posture interface for normal frontier operation. Group-domain navigation availability and forward walking remain separate from finding a bridge. The historical supervised trial produced stepping in place, not demonstrated forward navigation.

Final architecture: project frontier/mission coordinator -> existing university NavigateToPose -> existing university Nav2 planning/control -> smoother/mux -> existing motion bridge -> vendor locomotion. Preview remains the default. Required future deployment work is proving full university Nav2 bringup/action ownership, actual forward response, independent watchdog/stop enforcement and terrain-aware costs/policies. No additional transport is justified by the current evidence.

