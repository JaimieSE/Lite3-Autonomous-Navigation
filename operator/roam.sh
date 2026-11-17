#!/usr/bin/env bash
# M10 autonomous roaming: explore without goals, keep mapping, avoid obstacles, until you stop it.
# Independent of M0-M9 (own readiness and safety checks). DRY by default.
#
#   operator/roam.sh start                 dry: own readiness + what roaming WOULD do; nothing moves
#   operator/roam.sh start --physical      ARMED: starts navigation (type ARM) if needed, then type ROAM
#   operator/roam.sh start --sim           rehearsal against the software stand-in (no robot)
#   operator/roam.sh pause | resume | stop | status
#   operator/roam.sh speed slow|normal|fast|up|down
#   operator/roam.sh modes                 speed modes, and why gait modes are not used
# start options: --speed slow|normal|fast  --speed-limit M/S  --modes "slow=0.04,fast=0.09"
#   --geofence M (1-10, default 3)  --max-minutes N (<= 120)  --abort-clearance M  --frontier-radius M
#   --max-map-age S  --max-sensor-age S  --min-lidar-hz HZ  --direct-telemetry
#   --sim only: --sim-seconds S  --sim-script "60:pause;75:resume;90:speed:fast"
# Navigation is started for roaming with --clearance ${ROAM_NAV_CLEARANCE:-0.20} --max-speed ${ROAM_NAV_MAX_SPEED:-0.10}
# if it is not running. For speeds above 0.10: ROAM_NAV_MAX_SPEED=0.5 ROAM_NAV_CLEARANCE=0.15 operator/roam.sh start --physical --speed-limit 0.5
# THE HANDHELD REMOTE / E-STOP IS THE EMERGENCY STOP. "roam.sh stop" and Ctrl+C are commanded stops.
set -eo pipefail
source "$(dirname "$0")/env.sh"
ROAM_PID_FILE="$LOG_ROOT/roam.pid"
CMD="${1:-help}"; [ $# -gt 0 ] && shift

roam_running() { [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; }

case "$CMD" in
  start)
    PHYS=0; SIM=0
    for a in "$@"; do [ "$a" = --physical ] && PHYS=1; [ "$a" = --sim ] && SIM=1; done
    if roam_running; then echo "Roaming is already running (pid $(cat "$ROAM_PID_FILE")): operator/roam.sh status"; exit 3; fi
    if [ $SIM -eq 0 ]; then
      if [ -f "$PHYSICAL_PID_FILE" ] && kill -0 "$(cat "$PHYSICAL_PID_FILE")" 2>/dev/null; then
        echo "A physical test is running; finish or abort it first (operator/abort_physical.sh)."; exit 3; fi
      if [ -f "$TEST4_PID_FILE" ] && kill -0 "$(cat "$TEST4_PID_FILE")" 2>/dev/null; then
        echo "Test 4 is running; finish or abort it first (operator/abort_test4.sh)."; exit 3; fi
      if [ $PHYS -eq 1 ]; then
        if nav_running && [ "$(nav_mode)" != physical ]; then
          echo "REFUSED: navigation is running in $(nav_mode) mode. operator/stop_navigation.sh first."; exit 3; fi
        if ! nav_running; then
          echo "Starting navigation (PHYSICAL) for roaming..."
          "$OPERATOR_DIR/start_navigation.sh" --physical --clearance "${ROAM_NAV_CLEARANCE:-0.20}" --max-speed "${ROAM_NAV_MAX_SPEED:-0.10}" || exit 3
        fi
        echo "PHYSICAL ROAMING. Handheld remote / e-stop in your hand for the whole session."
      elif ! nav_running; then
        echo "Starting navigation (dry) for the roaming check..."
        "$OPERATOR_DIR/start_navigation.sh" --clearance "${ROAM_NAV_CLEARANCE:-0.20}" || exit 3
      fi
    fi
    python3 -m lite3_movement_tests.roam run "$@" 0<&0 &
    PID=$!
    echo "$PID" > "$ROAM_PID_FILE"
    trap 'kill -INT "$PID" 2>/dev/null' INT TERM
    set +e
    wait "$PID"; CODE=$?
    [ $CODE -gt 128 ] && { wait "$PID"; CODE=$?; }
    set -e
    rm -f "$ROAM_PID_FILE"
    exit $CODE ;;
  pause|resume|stop|status|modes)
    exec python3 -m lite3_movement_tests.roam "$CMD" ;;
  speed)
    [ $# -eq 1 ] || { echo "Usage: operator/roam.sh speed slow|normal|fast|up|down"; exit 2; }
    exec python3 -m lite3_movement_tests.roam speed "$1" ;;
  help|-h|--help)
    sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'
    exec python3 -m lite3_movement_tests.roam run --help ;;
  *) echo "Unknown command '$CMD' (start, pause, resume, stop, speed, status, modes, help)"; exit 2 ;;
esac
