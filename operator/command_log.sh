#!/usr/bin/env bash
# Robot command logger: records every command reaching the robot's motion_sender (domain 64) and every
# publisher joining/leaving the command topics in domains 64, 8 and 7. Read-only: it never publishes.
#
#   operator/command_log.sh start         run in the background (survives closing this terminal)
#   operator/command_log.sh stop | status
#   operator/command_log.sh show [-n N] [--all]   recent records (heartbeats hidden unless --all)
#   operator/command_log.sh run [--seconds S]     foreground, printing each record
#
# Records: log/operator/command-log/commands-YYYYMMDD.jsonl (one JSON object per line).
set -eo pipefail
source "$(dirname "$0")/env.sh"
OUT="$LOG_ROOT/command-log"
PID_FILE="$LOG_ROOT/command_log.pid"
export LITE3_COMMAND_LOG_DIR="$OUT"

running() { [ -f "$PID_FILE" ] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; }

CMD="${1:-status}"; shift || true
case "$CMD" in
  start)
    if running; then echo "Command logger already running (pid $(cat "$PID_FILE"))"; exit 0; fi
    mkdir -p "$OUT"
    nohup python3 -m lite3_movement_tests.command_log run --quiet "$@" >>"$OUT/logger.out" 2>&1 &
    echo $! > "$PID_FILE"
    sleep 3
    if running; then echo "Command logger running (pid $(cat "$PID_FILE")), records in $OUT"
    else echo "Command logger failed to start; see $OUT/logger.out"; rm -f "$PID_FILE"; exit 1; fi ;;
  stop)
    if running; then kill -INT "$(cat "$PID_FILE")"; sleep 2; fi
    running && kill "$(cat "$PID_FILE")" 2>/dev/null || true
    rm -f "$PID_FILE"; echo "Command logger stopped" ;;
  status)
    if running; then echo "Command logger: RUNNING (pid $(cat "$PID_FILE")), records in $OUT"
    else echo "Command logger: not running (operator/command_log.sh start)"; fi ;;
  show) exec python3 -m lite3_movement_tests.command_log show "$@" ;;
  run)  exec python3 -m lite3_movement_tests.command_log run "$@" ;;
  *) sed -n '2,10p' "$0"; exit 2 ;;
esac
