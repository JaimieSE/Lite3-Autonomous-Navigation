#!/usr/bin/env bash
# Abort a running Test 4 from another terminal (navigation keeps running, dry).
source "$(dirname "$0")/env.sh"
if [ -f "$TEST4_PID_FILE" ] && kill -0 "$(cat "$TEST4_PID_FILE")" 2>/dev/null; then
  kill -INT "$(cat "$TEST4_PID_FILE")" && echo "Abort sent to Test 4 (pid $(cat "$TEST4_PID_FILE"))."
else
  echo "Test 4 is not running."
fi
