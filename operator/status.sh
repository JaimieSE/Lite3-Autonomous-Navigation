#!/usr/bin/env bash
# Status of the navigation stack. Usage: operator/status.sh [--watch SECONDS]
set -eo pipefail
source "$(dirname "$0")/env.sh"
if nav_running; then
  grep -q '^BOUNDARY=.' "$STATE_FILE" && echo "Collision stop boundary: $(sed -n 's/^BOUNDARY=//p' "$STATE_FILE") m from the nose (clearance $(sed -n 's/^CLEARANCE=//p' "$STATE_FILE") m, <= $(sed -n 's/^MAX_SPEED=//p' "$STATE_FILE") m/s)"
  echo "Workflow: RUNNING ($(nav_mode) mode) since $(sed -n 's/^STARTED=//p' "$STATE_FILE"), logs $(sed -n 's/^RUN_DIR=//p' "$STATE_FILE")"
else
  echo "Workflow: not running (no live process group from operator/start_navigation.sh)"
fi
[ -f "$LOG_ROOT/roam.pid" ] && kill -0 "$(cat "$LOG_ROOT/roam.pid")" 2>/dev/null && \
  echo "M10 roaming: RUNNING (pid $(cat "$LOG_ROOT/roam.pid")); details: operator/roam.sh status"
MODE=$(nav_mode); exec python3 "$OPERATOR_DIR/lite3ops.py" status --mode "${MODE:-dry}" "$@"
