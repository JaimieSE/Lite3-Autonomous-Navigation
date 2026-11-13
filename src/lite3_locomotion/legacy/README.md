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

The original guard, motion sender and deterministic policies remain as Python
source for regression tests and safety reference. Their actuator/commissioning
console entry points are no longer installed. Their velocity watchdog and
handshake are not part of the university motion path.
