# Recovery candidate: build verification

Date: 2026-11-12. Environment: `group8`, x86_64, ROS 2 Humble, GCC 11.4.0,
C++14. This verifies compilation/packaging only, not physical operation.

| Check | Result |
| --- | --- |
| Existing vendor provenance | All 15 files match their SHA256 values in the pinned source manifest |
| Pinned commit | `e4ab5dc1c7f3ff46274162a1ef302253936578aa` |
| Existing interface definitions | Installed MotionSimpleCMD and MotionComplexCMD match the vendor definitions byte-for-byte |
| Existing interface prefix | `/home/user/lite3_ws/install/transfer_interfaces` |
| Untouched vendor CMake configure | Passed with existing dependencies |
| Untouched vendor motion compilation | Failed at missing `tf2_geometry_msgs/tf2_geometry_msgs.h` include path |
| Recovery wrapper configure | Passed with explicit tf2_geometry_msgs, TF/message and Threads dependencies |
| Recovery motion compile/link | Passed; original C++ and protocol header unchanged |
| Isolated install | Passed; every install-manifest destination is inside the candidate prefix |
| Read-only package-index inspection | Reports `transfer jetson2motion` when only that inspection process includes the candidate prefix |
| Normal underlay package lookup afterward | `transfer` remains not found; candidate not activated in the normal environment |
| Installed vendor launch/app/checker | Absent by design |
| Ordinary workspace discovery | Lists only lite3_locomotion, livox_ros_driver2 and existing transfer_interfaces; recovery candidate is ignored |
| Executable architecture | ELF64, little-endian x86-64 |
| Binary inspection | ELF metadata and NEEDED dependencies inspected without executing the bridge |
| Motion-host connection or robot packets | None from this preparation; bridge was never executed |
| Mode/posture/service changes | None |
| Firmware, ARM/Jazzy, stopping behavior | Not verified |

## Output locations

- [Build tree](../../.colcon/recovery_transfer/build/).
- [Install manifest](../../.colcon/recovery_transfer/build/install_manifest.txt).
- [Isolated package prefix](../../.colcon/recovery_transfer/install/transfer/).
- Installed executable: `lib/transfer/jetson2motion` within that prefix.
- Installed executable SHA256:
  `cd279e59cc33ac4c5a6484dca38864c7324c1d47e8f0ee214d750a0624ab0a7a`.
- The pristine upstream failure check is separate, under
  `.colcon/recovery_transfer/upstream_build` in the locomotion package. Nothing
  from that baseline was installed into a university prefix.

The installed executable's runtime search path is empty. It still needs the
approved Humble/transfer_interfaces runtime environment if execution is later
authorized. Read-only package discovery does not establish dynamic loading,
runtime behavior, robot compatibility or emergency stopping. No `ros2 run`,
`ros2 launch`, live ROS integration test, packet probe or bridge startup was used.

## Warnings retained from original code

- Deprecated `tf2_geometry_msgs.h` forwarding header.
- Anonymous struct extensions under `-Wpedantic`.
- `%d` format arguments receiving `ssize_t` values.
- Unused `sendto` result variables in simple/complex callbacks.

The wrapper intentionally does not hide or patch these warnings. More important
runtime issues remain: no command-expiry stop in the original sender, unbounded
blocking receive/shutdown, no startup arm, and no robot-mode or physical-stop
acknowledgment. These are deployment review blockers, not compile failures.

## Scope of the result

The answer to "can the historical motion target compile on this Humble host
against the existing interfaces?" is **yes, with the isolated build metadata
fixes supplied here**. The answer to "is it an approved, deployable, stop-safe
physical control installation?" remains **no**.

See the [build plan and prerequisites](README.md) and
[unresolved deployment record](deployment_review.yaml). The candidate must not
be substituted into the live university environment on the strength of this
build result alone.
