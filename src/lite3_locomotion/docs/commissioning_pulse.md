> Historical standalone-stack workflow. Its motion entry points are retired from
> the current installation. Use the [current architecture](autonomy_stack.md)
> and [change audit](simplification_audit.md); these earlier steps are not the
> current startup procedure.

# Single-process commissioning pulse

## Why this runner exists

The earlier two-CLI physical attempt reached `motion_sender`: 20 requests at
0.03 m/s over approximately 0.960 s, with successful 20-byte forward UDP sends.
The operator observed no walking. It did not establish robot acceptance or
stopping. Separate publisher processes introduced a **1.436 s gap** between
the last forward callback and first zero, so that procedure must not be reused.

[commissioning_pulse.py](../legacy/lite3_legacy/commissioning_pulse.py) replaces
that sequence with one publisher and monotonic deadlines. It is a manual
commissioning tool, **not** a LiDAR collision guard or autonomy integration.
It never changes control mode, gait or posture and never automatically sits.

## Behavior

- Defaults to `/lite3_locomotion/pulse_test_cmd_vel`, not `/cmd_vel`.
- Fixed 0.03 m/s requested forward speed, 1 s deadline and nominal 20 Hz.
- Starts with 0.5 s of zeros, then forward, then an immediate zero using the
  **same publisher**, followed by at least 2 s of zero requests.
- Cancellation, unhealthy graph or a scheduler gap >=0.15 s latches off the
  remaining forward phase. No automatic retries or rearming.
- SIGINT/SIGTERM request cancellation; cleanup attempts zeros. Exceptions
  likewise attempt zeros. Observer errors cannot cancel the remaining attempts.
- A blocking operation, SIGKILL, crash, host/network failure or robot rejection
  can still defeat timely stopping. This is not a hard-real-time guarantee or
  an independent watchdog.
- Physical opt-in uses domain 8 `/cmd_vel`; a separate observer thread in domain
  64 records command arrivals, including forward-to-zero timing. It checks for
  the known sender/bridge and conflicting velocity publishers. ROS graph names
  are wiring checks, not authorization or robot-state acknowledgment.
- Observed ROS zeros and UDP send success are not proof of physical stop.

## Run without physical output

This source command works without rebuilding the installed package:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="/home/user/lite3_ws/src/lite3_locomotion:$PYTHONPATH"
/usr/bin/python3 -m lite3_locomotion.commissioning_pulse
```

Expect JSON identifying `physical: false`, the separate test topic, message
counts and monotonic timing. It must never be remapped to a motor input.
The new `commissioning_pulse` console entry point becomes available only after
a normal reviewed package build; no build/activation is required for the source
command above.

## Isolated validation performed

On 2026-11-12:

- 14 deterministic policy/argument tests plus one real ROS loopback test passed
  (15 total). Tests cover the deadline, cancellation, scheduler delay, slow
  health checking, exception cleanup, bounds and explicit physical flags.
- ROS integration used domain 97, localhost-only Fast DDS and the existing
  loopback transport profile; it did not connect to the physical bridge.
- Receiver first-forward to first-zero: **1.000089 s**.
- Receiver last-forward to first-zero: **0.046436 s**.
- Local first-forward to first-zero: **1.000092 s**.
- Timing measurements are one local run, not worst-case guarantees.

To repeat:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="/home/user/lite3_ws/src/lite3_locomotion:$PYTHONPATH"
ROS_LOCALHOST_ONLY=1 \
RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/lite3_ws/src/lite3_locomotion/test/loopback_dds.xml \
FASTDDS_DEFAULT_PROFILES_FILE=/home/user/lite3_ws/src/lite3_locomotion/test/loopback_dds.xml \
LITE3_RUN_PULSE_LOOPBACK=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONDONTWRITEBYTECODE=1 \
/usr/bin/python3 -m pytest -s -q -p no:cacheprovider \
  /home/user/lite3_ws/src/lite3_locomotion/test/test_commissioning_pulse.py
```

These environment overrides apply only to the test process. Do not configure
physical operation to use localhost-only discovery or test domain 97.

## Physical retry: prepared, not executed

The current app home screen does **not** establish Navigation mode. A retained
Auto command is not mode acknowledgment. The remaining no-walking cause has not
been established; possibilities include control state or firmware acceptance.
Do not raise speed or change gait to work around it without evidence.

Physical execution requires `--physical` and all of the following explicit
flags: `--navigation-confirmed`, `--operator-ready`, `--standing-confirmed`,
`--exclusive-control`, `--accept-no-lidar`. They represent fresh operator
confirmations, not automatic state measurements. Do not pass them merely to
silence a preflight error.

Before another physical test:

1. Identify and confirm Navigation/Auto state through the actual control
   interface; home-screen visibility is insufficient. Mode changes are separate
   authorized operations, not part of this runner.
2. Confirm standing, level ground, clear path, exclusive control and the tested
   STOP control in the nearby operator's hand. Keep it ready throughout.
3. Prepare System motion-driver log capture and operator observation; the
   runner's domain-64 observer captures ROS delivery, not packets at the robot.
4. Authorize one test only. Record local timing, domain-64 timing/counts,
   `motion_sender` UDP logs and the operator's actual movement/stop observation.
5. If no movement occurs despite observed commands, do not auto-retry or
   increase speed. Diagnose robot state/firmware next.
6. Confirm physical stop before considering any separate sit toggle. The tool
   always reports `physical_stop_verified: false`; only external evidence can
   establish that.

This tool does not finish the guarded university-bridge integration described
in the [daily checklist](daily_startup_checklist.md).
