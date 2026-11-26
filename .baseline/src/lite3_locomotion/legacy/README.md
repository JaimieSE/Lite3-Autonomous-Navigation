# Retired standalone stack fixtures

These configuration, launch and integration files belonged to the duplicated
project Nav2/guard/UDP stack. They were archived after the replacement
NavigateToPose contract suite passed (17 tests on 2026-11-13). They are excluded
from package installation, colcon discovery and ordinary pytest collection.

They depend on the original coordinator, CLI and installed executables. Do not
run them against the refactored installation. To reproduce that historical
stack, use branch `preserve/pre-university-reuse-2026-11-13` in a separate checkout
and an isolated loopback test environment. Archiving does not validate physical
bridge behavior or authorize motion.

## Retained policy and simulation tools

The six retired modules now live under [lite3_legacy](lite3_legacy/). They are
outside the explicitly installed `lite3_locomotion` package. The active package
does not import this namespace. Their actuator/commissioning console entry
points are not installed. Their velocity watchdog and handshake are not part
of the university motion path. The shared scan predicate lives in active
[safety.py](../lite3_locomotion/safety.py).

The five **offline** policy/protocol suites have been updated for the relocated
namespace and remain supported. From the package root, run:

```bash
PYTHONPATH="$PWD/legacy:$PWD:${PYTHONPATH:-}" \
  python3 -m pytest -q -p no:cacheprovider \
  legacy/test/test_bounded_test.py legacy/test/test_commissioning_pulse.py \
  legacy/test/test_motion_sender.py legacy/test/test_movement_guard.py \
  legacy/test/test_protocol_preview.py
python3 legacy/tools/preview_movement_test.py
```

The trace generator writes simulated intent sequences only. The ordinary pulse
suite skips its opt-in ROS timing test; no physical command runner is invoked.
Protocol framing uses a local loopback socket. These tests do not commission a
robot bridge. The older end-to-end Nav2/sender fixtures in this directory still
require the original branch and are not part of this supported offline command.

The excluded commissioning tool retains a one-second default and now accepts
an explicit `--duration` up to two seconds for the separately authorized
supervised trial. Its maximum requested speed remains 0.03 m/s. Both durations
passed deterministic and isolated ROS timing tests. It does not infer posture,
prove physical rest, automatically toggle stand/sit, or constitute autonomous
navigation. See the [two-second trial record](../docs/walking_trial_two_seconds_2026-11-13.md).
