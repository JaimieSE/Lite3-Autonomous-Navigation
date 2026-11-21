#!/usr/bin/env bash
# Stop Test 4 / physical tests (if running), cancel Nav2 goals, then stop ONLY the processes this workflow started.
# Logs are kept. University services and other users' processes are not touched.
set -o pipefail
source "$(dirname "$0")/env.sh"

ROAM_PID_FILE="$LOG_ROOT/roam.pid"
if [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; then
  echo "== Stopping M10 roaming (it cancels Nav2, commands zeros and verifies rest first)"
  python3 -m lite3_movement_tests.roam stop || kill -INT "$(cat "$ROAM_PID_FILE")"
  for _ in $(seq 1 40); do kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null || break; sleep 0.5; done
fi
rm -f "$ROAM_PID_FILE"

PGIDS=$(workflow_pgids)
if [ -z "$PGIDS" ]; then
  echo "Navigation is not running (nothing of this workflow to stop)."
  rm -f "$STATE_FILE"
  exit 0
fi
echo "== Cancelling Nav2 goals"
timeout 15 python3 "$OPERATOR_DIR/lite3ops.py" cancel-goals || true

echo "== Stopping process group(s): $(echo $PGIDS)"
for g in $PGIDS; do kill -INT -- "-$g" 2>/dev/null; done
for _ in $(seq 1 20); do
  alive=""; for g in $PGIDS; do pgrep -g "$g" >/dev/null && alive="$alive $g"; done
  [ -z "$alive" ] && break; sleep 0.5
done
if [ -n "$alive" ]; then
  # Known: the Nav2 component container often ignores SIGINT. twist_mux (the only command path)
  # has already exited by now; escalate for the rest.
  echo "   still running after 10 s ($(ps -o comm= -g "$(echo $alive | tr ' ' ',')" 2>/dev/null | sort -u | tr '\n' ' ')); sending SIGTERM"
  for g in $alive; do kill -TERM -- "-$g" 2>/dev/null; done; sleep 5
  for g in $alive; do pgrep -g "$g" >/dev/null && { echo "   SIGKILL group $g"; kill -KILL -- "-$g"; }; done
fi
for g in $PGIDS; do
  for _ in $(seq 1 10); do pgrep -g "$g" >/dev/null || break; sleep 0.5; done   # let the kernel reap
  if pgrep -g "$g" >/dev/null; then
    echo "ERROR: process group $g survived. Remaining:"; ps -o pid,args -g "$g"
    echo "Recovery: kill -KILL -- -$g   (these are only this workflow's processes)"; exit 1
  fi
done
RUN_DIR=$(sed -n 's/^RUN_DIR=//p' "$STATE_FILE" 2>/dev/null)
rm -f "$STATE_FILE"
echo "   all workflow processes stopped"
timeout 40 python3 "$OPERATOR_DIR/lite3ops.py" graph-clear --timeout 25 || true
echo "Stopped. Logs kept in: ${RUN_DIR:-$LOG_ROOT}"
