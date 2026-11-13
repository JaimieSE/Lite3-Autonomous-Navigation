# lite3_locomotion

**2026-11-13 reuse audit:** see the [simplification audit](docs/simplification_audit.md)
for retained functionality, duplicate-stack candidates, proposed architecture,
test results and migration prerequisites. Runtime refactoring is pending access
to the actual university source and a Git checkout in which the original can be
preserved. The implementation documented below is unchanged; university action
compatibility and bridge replacement have not been verified.

## Nav2 and guarded adapter

The new [autonomy stack](docs/autonomy_stack.md) provides Nav2 configuration,
a bounded frontier coordinator, and a separate guarded command adapter. It
starts **dry-run and disarmed**. Follow that guide for build, isolated synthetic
verification, live-map dry-run, and software stop commands. No physical movement
or robot mode change has been performed. Manual-mode integration is omitted.

The sections below describe the original non-actuating test controller.

For the new opt-in UDP movement sender, commissioning prerequisites and explicit
robot-output workflow, see [robot output](docs/robot_output.md). It starts disarmed
and has only been tested without physical robot movement.

## Exploration preview

The [RViz exploration preview](docs/exploration_preview.md) can suggest frontier
goals from the existing live map without a preplanned route. It publishes only
`/lite3_locomotion/exploration_markers`; yellow points are candidates and a green
arrow is the selected suggestion. No navigation goal or robot command is sent.
See the linked instructions to display it in your domain-8 RViz instance.

ROS 2 Humble `ament_python` package for **non-actuating command-policy testing**.
The test controller has no SDK, UDP client, motion-host connection, gait/mode
command, posture command, or real robot bridge. Do not connect its test topics
to an actuator or command relay. A pinned vendor source copy is staged separately
for review and excluded from ordinary build/discovery; see the
[movement-test review](docs/movement_test_review.md).

## Architecture

```text
Synthetic Twist input: /lite3_locomotion/test_nav_cmd_vel
    -> validation -> symmetric velocity limits -> monotonic timeout / stop state
    -> processed Twist output: /lite3_locomotion/test_cmd_vel
```

[command_policy.py](lite3_locomotion/command_policy.py) contains the ROS-independent
logic. [controller_node.py](lite3_locomotion/controller_node.py) adapts that policy
to two fixed test topics. Output is published at 10 Hz, including zeros while
waiting, stopped, invalid, or timed out. Invalid input also publishes zero
immediately from its callback. The timer observes timeout at the next callback;
scheduling delays can extend this beyond the nominal 0.1-second timer period.
This is not a real-time or certified safety controller.

## Placeholder configuration

| Application argument | Default | Meaning |
| --- | --- | --- |
| `--max-linear-x` | 0.10 | Symmetric forward/reverse bound, interpreted as m/s |
| `--max-linear-y` | 0.05 | Symmetric lateral bound, interpreted as m/s |
| `--max-angular-z` | 0.20 | Symmetric yaw bound, interpreted as rad/s |
| `--command-timeout` | 0.50 | Receipt-time expiry in monotonic seconds |

These values are for offline tests only. They are not verified Lite3 limits or
proof that the real controller supports lateral motion or these units.
Limits must be finite and nonnegative; zero disables an axis. Timeout must be
finite and strictly positive. Arguments configure only the policy, never topics,
namespaces, robot addresses, SDK modes, or actuator outputs.

Commands use body-frame x/y velocity and yaw rate by convention. Every component
must be finite. Nonzero `linear.z`, `angular.x`, or `angular.y` rejects the entire
command. Supported finite components are clamped, not rejected for exceeding a
bound. A zero command is valid.

## Freshness and stop states

Startup state is `WAITING` with zero output. An accepted command enters `ACTIVE`.
Invalid input clears the previous command and enters `INVALID`. At elapsed time
greater than or equal to the timeout, the policy clears it and enters
`TIMED_OUT`. A fresh valid command recovers from either condition.

`set_stop(True)` latches `STOPPED`, clears the command and rejects further input.
`set_stop(False)` returns to `WAITING`, requiring a new command. The node exposes
two fixed `std_srvs/srv/Trigger` test services:

| Service | Behavior |
| --- | --- |
| `/lite3_locomotion/test_kill` | Latch software stop, clear the stored command, immediately publish test zero, and reject further velocity input |
| `/lite3_locomotion/test_reset_kill` | Explicitly release the latch; require a fresh command before accepting nonzero output |

Repeated kill calls keep the latch set; they never toggle it off. The latch lasts
for the running process only. Restart starts at `WAITING` again; a future real
adapter must have a separate startup arming and independent watchdog mechanism.
Service callbacks run on the same single-thread executor as the control loop:
they cannot handle a hung process, dead network, or failed computer. A successful
response acknowledges only local software handling, not physical stopping.

For a manually started **isolated test instance**, use a terminal with the same
test-domain and loopback-only DDS environment. These commands address test
services only:

```bash
ros2 service call /lite3_locomotion/test_kill std_srvs/srv/Trigger '{}'
# Resume only after an intentional reset; previous commands are not replayed:
ros2 service call /lite3_locomotion/test_reset_kill std_srvs/srv/Trigger '{}'
```

The automated runner starts and stops its own instance, so these services are
not left running afterward. Do not interpret this software kill as a hardware
emergency stop or handheld-controller takeover. No Manual-mode command is sent.

Shutdown attempts one zero publication if its context
is still valid. Delivery at shutdown is not guaranteed; process termination or
loss of power cannot be handled by this node.

`Twist` has no timestamp. Expiry measures time since callback receipt, not the
age of the original navigation calculation. The subscription queue depth is one
but does not prove end-to-end freshness. A future actuator integration requires
its own independent, verified command watchdog.

## Topic isolation

The executable rejects ROS arguments (including `--ros-args`, remappings, and
parameter files) before initializing ROS. It initializes a separate context with
an empty argument list, disables global arguments for its node, and fixes its
namespace to `/`. Resolved topic and service names are checked before creating endpoints.
There are no topic arguments, ROS configuration parameters, launch files, or
actuating-mode switches. Launch mechanisms that inject ROS arguments cause
argument parsing to fail rather than redirect this node.

The only processed-command publisher is `/lite3_locomotion/test_cmd_vel`.
There is no publisher to `/cmd_vel`, `/cmd_vel_out`, `/simple_cmd`, or any SDK
interface. Internal ROS middleware traffic is still present when the executable
runs. Topic names are not a security boundary: an external subscriber or relay,
modified source, or hostile middleware can defeat isolation. Run any eventual
interactive test in a dedicated test environment with no actuator subscribers.
The package does not discover, connect to, or validate such external consumers.

## Offline tests and isolated build

From this package directory, run the tests without starting ROS or DDS:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider test
```

Tests use a fake monotonic clock and fake ROS endpoints. They cover all message
components, finite/unsupported input validation, symmetric and custom limits,
expiry boundaries and recovery, stop latching and release, argument rejection,
and resolved-topic guards. They do not establish physical robot behavior.

To build and test with existing Humble dependencies while keeping generated
files inside this package directory, run from this directory:

```bash
mkdir -p .colcon
touch .colcon/COLCON_IGNORE
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$PWD/.colcon/log" build \
  --base-paths "$PWD" --packages-select lite3_locomotion \
  --build-base "$PWD/.colcon/build" --install-base "$PWD/.colcon/install"
PYTHONDONTWRITEBYTECODE=1 colcon --log-base "$PWD/.colcon/log" test \
  --base-paths "$PWD" --packages-select lite3_locomotion \
  --build-base "$PWD/.colcon/build" --install-base "$PWD/.colcon/install" \
  --pytest-args -p no:cacheprovider
colcon test-result --test-result-base "$PWD/.colcon/build" --verbose
```

No dependency installation is required or performed. Build output is not added
to the workspace's existing build/install directories. These ordinary unit tests
do not start the ROS executable or publish commands.

## Opt-in ROS integration test

After the isolated build above, run from this package directory:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 tools/run_ros_integration.py
```

This starts the **installed test controller** and a test observer/publisher. It
does not start Nav2, a driver, a robot service, or an SDK bridge. The runner sets
`ROS_LOCALHOST_ONLY=1`, forces Fast DDS with the checked-in
[loopback transport profile](test/loopback_dds.xml), disables built-in transports
(including shared memory), and permits UDP on `127.0.0.1` only. It selects a
domain in 40–89, separately from the audited robot domain 8, clears discovery
server overrides, and rejects discovered unexpected nodes before controller
startup. During tests it checks node names, topics, and test-topic endpoint
owners before sending synthetic commands. An occupied domain fails the suite;
it does not connect to or modify the occupants.

The suite verifies startup zero output, approximate periodic output, valid
pass-through, positive/negative limiting, NaN and unsupported-axis rejection,
timeout fallback/persistence/recovery, real-executable remapping rejection,
kill latching under continuing commands, and reset without stale replay.
The timeout test allows a scheduling margin; it does not establish a real-time
deadline. The kill/reset tests call only the two fixed test services.

Ordinary pytest/colcon runs **skip** these eleven integration cases. Use the runner
to opt in; it verifies the isolation settings before importing or initializing
ROS. Results are written under `.colcon/integration/`: `results.xml` (JUnit
pass/fail report), `pytest.log`, `controller.log`, and ROS logs. Child processes
are stopped on completion; the runner has a 90-second suite timeout and process
group cleanup for interruption. No ROS daemon is launched or used.

Loopback isolation assumes a normally functioning middleware and no hostile
local forwarding process. Graph discovery cannot prove the absence of an
undiscovered or spoofed consumer. This checks the test interface; it does not
establish robot safety or authorize connecting an actuator.

## Earlier bridge preparation and remaining deployment work

The [bounded movement-test preparation](docs/movement_test_review.md) includes
the official bridge source, a Humble compilation wrapper, and a one-shot
[offline sequencer](lite3_locomotion/bounded_test.py). It generates velocity and
Manual-mode request intents only. The new separate adapter is described in the [autonomy guide](docs/autonomy_stack.md).
The offline sequencer remains unchanged; no automatic mode switching or
connection to the test controller's kill service is implemented.

The [vendor source audit](docs/vendor_interface_audit.md) identifies the official
`Lite3_ROS` bridge, its source commit, packet mapping, and remaining deployment
and manual-override gaps. [protocol_preview.py](lite3_locomotion/protocol_preview.py)
is an offline, bytes-only prototype of that mapping, with packet-layout tests.
It is not connected to the test controller and cannot transmit commands.

The intended future architecture is Nav2 -> command arbitration/manual override
-> validation and watchdog -> verified Lite3 adapter -> vendor locomotion
controller. Keep one authoritative odometry/TF chain and add terrain assessment
from LiDAR/IMU to influence planner costs and speed limits. This initial package
does not yet assess terrain, choose gaits, or control joints.

Before separately approving an actuator integration, locate the deployed
`transfer/jetson2motion` implementation and domain-bridge configuration. Confirm
command topics and direction, source/destination domains, QoS, body axes/signs,
SI units or scaling, command codes, firmware compatibility, rate requirements,
independent watchdog, loss-of-link behavior, stop semantics, and manual override.
Historical launch defaults are not sufficient evidence. The currently observed
telemetry bridge is not a verified motion interface. The mux's configured
0.5-second input expiry is not proof of a controller stop timeout.

Review the separate guarded adapter and deployment configuration against
that evidence before physical commissioning. Do not turn this test harness into an actuating
node by adding a topic override or a mode flag.
