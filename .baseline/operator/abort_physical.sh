#!/usr/bin/env bash
# Abort running physical tests from another terminal: the runner cancels the Nav2 goal,
# commands zero velocity (above all locks), verifies rest and releases nothing.
# THIS IS NOT AN EMERGENCY STOP. If the robot does not stop, use the handheld remote / e-stop.
source "$(dirname "$0")/env.sh"
if [ -f "$PHYSICAL_PID_FILE" ] && kill -0 "$(cat "$PHYSICAL_PID_FILE")" 2>/dev/null; then
  kill -INT "$(cat "$PHYSICAL_PID_FILE")" && echo "Abort sent to the physical test (pid $(cat "$PHYSICAL_PID_FILE"))."
else
  echo "No physical test is running."
fi
if [ -f "$LOG_ROOT/roam.pid" ] && kill -0 "$(cat "$LOG_ROOT/roam.pid")" 2>/dev/null; then
  kill -INT "$(cat "$LOG_ROOT/roam.pid")" && echo "Stop sent to M10 roaming (pid $(cat "$LOG_ROOT/roam.pid"))."
fi
