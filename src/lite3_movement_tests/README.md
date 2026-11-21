# lite3_movement_tests: autonomous roaming (M10)

Roaming explores on its own until stopped: own readiness checks (telemetry, Nav2/SLAM, terrain monitor,
LiDAR, command path, stand), then scan-walk exploration through Nav2, with an independent watchdog.
The M0-M9 physical test suite was removed on 2026-11-21 (copies in `.baseline/`).

Run it with `operator/roam.sh` (see `operator/README.md` and `operator/CHEATSHEET.md`):

```bash
operator/roam.sh start --sim                    # simulator, no robot
operator/roam.sh start --physical --max-speed 0.9   # robot MOVES; type ARM, then ROAM
operator/roam.sh pause | resume | stop | status | speed fast | gait rug_hstep
```

Modules: `roam.py` (session, scan-walk, CLI), `roam_ros.py` / `roam_sim.py` (robot interfaces),
`roam_watchdog.py`, plus the shared readiness runner (`runner.py`, `nav_steps.py`, `m4_steps.py`),
`ros_io.py`, `telemetry.py`, `geometry.py`, `config.py`, `sim_model.py`, `proximity.py`, `report.py`.
Tests: `python3 -m pytest test` (no ROS, no robot).
