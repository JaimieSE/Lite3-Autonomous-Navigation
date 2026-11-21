# Lite3 roaming cheat sheet (remote / STOP in hand whenever the robot is armed)

```bash
cd ~/lite3_ws && export LITE3_LIO_SOURCE=local
operator/build.sh                                  # after code changes
operator/roam.sh start --sim                       # rehearse, no robot
operator/roam.sh start --physical --max-speed 0.9  # type ARM (navigation), then ROAM
operator/roam.sh speed fast                        # use the session maximum
operator/roam.sh pause | resume | stop | status
operator/roam.sh gait rug_general|rug_grip|rug_hstep|flat_slow|flat_medium|flat_fast   # while paused
operator/stop_navigation.sh                        # stop the whole stack
operator/start_navigation.sh --full-stops          # older extra stop layers (default: docs-only stops)
```

Robot: standing, Navigation (auto) mode on the remote, sticks untouched, RViz closed.
Emergency stop: the remote STOP button (or turn auto mode off).
