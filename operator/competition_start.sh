#!/usr/bin/env bash
# One command for a competition roam (operator 2026-11-24): readiness check, navigation, Pi mission bridge, RViz with
# every roam indicator, then roam with the unified decision (course grid memory, one evaluator, stuck ladder with Pi
# scene understanding, pauses that resume by themselves). Nothing moves until you type ARM and ROAM.
#
#   operator/competition_start.sh --physical      ARMED run: starts navigation (type ARM), then roam (type ROAM);
#                                                 after ROAM it puts the robot in navigation (auto) mode itself
#   operator/competition_start.sh                 dry: real sensors, robot still; RViz shows what roam would do
#   operator/competition_start.sh --sim           rehearsal against the software stand-in (no robot, no RViz)
#   operator/competition_start.sh status          roam status and the helpers this command started
#   operator/competition_start.sh stop            stop roam, the helpers, then navigation
#
# Options (before any roam option):
#   --missions FILE   missions JSON for the Pi mission bridge (default ~/live_mission/competition-v6.3-practice.json)
#   --no-pi-bridge    do not start ~/live_mission/pi_bridge.sh (dry: it only decides M01-M05, never moves the robot)
#   --no-rviz         do not open RViz (operator/competition.rviz)
# When roam ends, the Pi bridge stops; RViz and navigation stay up ("stop" ends navigation too).
# Every other option goes to roam (operator/roam.sh start), e.g. --max-minutes 30, --auto-resume 20,
# --weights "grid=2", --geofence 10. Speed is normal (regular) by default.
# THE HANDHELD REMOTE / E-STOP IS THE EMERGENCY STOP.
set -eo pipefail
source "$(dirname "$0")/env.sh"
PIDS_FILE="$LOG_ROOT/competition.pids"
ROAM_PID_FILE="$LOG_ROOT/roam.pid"
export LITE3_LIO_SOURCE="${LITE3_LIO_SOURCE:-local}"

helpers_stop() {
  [ -f "$PIDS_FILE" ] || return 0
  while read -r name pid; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      echo "   stopping $name (pid $pid)"
      kill -INT "$pid" 2>/dev/null || true
      for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
      kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
    fi
  done < "$PIDS_FILE"
  rm -f "$PIDS_FILE"
}

case "${1:-}" in
  stop)
    echo "== Competition stop"
    if [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; then
      "$OPERATOR_DIR/roam.sh" stop || true
      for _ in $(seq 1 40); do kill -0 "$(cat "$ROAM_PID_FILE" 2>/dev/null)" 2>/dev/null || break; sleep 0.5; done
    fi
    helpers_stop
    exec "$OPERATOR_DIR/stop_navigation.sh" ;;
  status)
    "$OPERATOR_DIR/roam.sh" status || true
    if [ -f "$PIDS_FILE" ]; then
      while read -r name pid; do
        if kill -0 "$pid" 2>/dev/null; then echo "$name: running (pid $pid)"; else echo "$name: not running"; fi
      done < "$PIDS_FILE"
    fi
    echo "navigation: $(nav_running && echo "running ($(nav_mode))" || echo 'not running')"
    exit 0 ;;
  -h|--help|help)
    sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
esac

MODE=dry; PI_BRIDGE=1; RVIZ=1
MISSIONS="$HOME/live_mission/competition-v6.3-practice.json"
ROAM_ARGS=()
while [ $# -gt 0 ]; do
  a="$1"; shift
  case "$a" in
    --physical) MODE=physical; ROAM_ARGS+=(--physical --auto-mode on) ;;   # robot to navigation (auto) mode
    --sim) MODE=sim; ROAM_ARGS+=(--sim) ;;
    --dry) MODE=dry ;;
    --missions) MISSIONS="$1"; shift ;;
    --no-pi-bridge) PI_BRIDGE=0 ;;
    --no-rviz) RVIZ=0 ;;
    *) ROAM_ARGS+=("$a") ;;
  esac
done
if [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; then
  echo "Roam is already running (pid $(cat "$ROAM_PID_FILE")): operator/competition_start.sh status | stop"; exit 3
fi
# The unified decision and self-resuming pauses are roam's defaults; say so explicitly for the record.
case " ${ROAM_ARGS[*]} " in *" --decision "*) ;; *) ROAM_ARGS+=(--decision unified) ;; esac
case " ${ROAM_ARGS[*]} " in *" --speed "*) ;; *) ROAM_ARGS+=(--speed normal) ;; esac

RUN="$LOG_ROOT/competition-$(date +%Y%m%d-%H%M%S)-$MODE"
mkdir -p "$RUN"
echo "== Competition start ($MODE). Logs: $RUN"

if [ "$MODE" != sim ]; then
  echo "== Readiness check (read-only)"
  "$OPERATOR_DIR/preflight.sh" --direct-telemetry 2>&1 | tee "$RUN/preflight.txt" | tail -15 || true
  # roam.sh refuses a physical run on a dry navigation stack (and the reverse is pointless): restart it
  if nav_running && [ "$(nav_mode)" != "$MODE" ]; then
    echo "== Navigation is running in $(nav_mode) mode; stopping it for a $MODE start"
    "$OPERATOR_DIR/stop_navigation.sh"
  fi
  : > "$PIDS_FILE"
  if [ $PI_BRIDGE -eq 1 ]; then
    if [ -x "$HOME/live_mission/pi_bridge.sh" ] && [ -f "$MISSIONS" ]; then
      echo "== Pi mission bridge (dry decisions M01-M05) -> $RUN/pi_bridge.log"
      setsid "$HOME/live_mission/pi_bridge.sh" --dry-run --missions "$MISSIONS" --out "$RUN/pi_bridge" \
        > "$RUN/pi_bridge.log" 2>&1 < /dev/null &
      echo "pi_bridge $!" >> "$PIDS_FILE"
    else
      echo "   NOTE: Pi mission bridge not started (missing ~/live_mission/pi_bridge.sh or $MISSIONS)"
    fi
  fi
  if [ $RVIZ -eq 1 ]; then
    if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v rviz2 >/dev/null; then
      echo "== RViz (operator/competition.rviz): map, costmap, plan, roam grid/goal/alternatives/status line"
      setsid rviz2 -d "$OPERATOR_DIR/competition.rviz" > "$RUN/rviz.log" 2>&1 < /dev/null &
      echo "rviz $!" >> "$PIDS_FILE"
    else
      echo "   NOTE: no display here; open RViz yourself: rviz2 -d $OPERATOR_DIR/competition.rviz"
    fi
  fi
fi

on_exit() {
  code=$?
  echo "== Roam ended (exit $code). Stopping the Pi mission bridge; RViz and navigation stay up."
  if [ -f "$PIDS_FILE" ]; then
    grep -v '^rviz ' "$PIDS_FILE" > "$PIDS_FILE.keep" || true
    mv "$PIDS_FILE.keep" "$PIDS_FILE"
    helpers_stop
  fi
  echo "   Reports: $(ls -dt "$LOG_ROOT"/run-*/roam/*/ 2>/dev/null | head -1) (report.md, mission_events.json)"
  [ "$MODE" != sim ] && echo "   Pi bridge: $RUN/pi_bridge/   End everything: operator/competition_start.sh stop"
  return 0
}
trap on_exit EXIT

echo "== Roam: ${ROAM_ARGS[*]}"
"$OPERATOR_DIR/roam.sh" start "${ROAM_ARGS[@]}"
