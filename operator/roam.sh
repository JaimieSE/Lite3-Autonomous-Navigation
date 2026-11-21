#!/usr/bin/env bash
# M10 autonomous roaming: explore without goals, keep mapping, avoid obstacles, until you stop it.
# Independent of M0-M9 (own readiness and safety checks). DRY by default.
#
#   operator/roam.sh start                 dry: own readiness + what roaming WOULD do; nothing moves
#   operator/roam.sh start --physical      ARMED: starts navigation (type ARM) if needed, then type ROAM
#   operator/roam.sh start --sim           rehearsal against the software stand-in (no robot)
#   operator/roam.sh pause | resume | stop | status
#   operator/roam.sh speed slow|normal|fast|up|down
#   operator/roam.sh gait rug_general|rug_grip|rug_hstep|flat_slow|flat_medium|flat_fast   (while paused)
#   operator/roam.sh stairs up|down        cross stairs (from PAUSED, facing them; high-step gait)
#   operator/roam.sh modes                 speed modes and gaits
#   operator/roam.sh plan [--max-speed V]  effective speed limits, stop zones, turning clearance (no motion)
# Default strategy scan-walk: walk an observed open corridor; search left/right in short turns when no route is visible.
# start options: --max-speed V (default 0.50)  --strategy scan-walk|frontier  --speed slow|normal|fast
#   --speed-limit M/S  --modes "slow=0.04,fast=0.09"
#   --geofence M (1-10, default 10)  --max-minutes N (<= 120)  --abort-clearance M  --frontier-radius M
#   --max-map-age S  --max-sensor-age S  --min-lidar-hz HZ  --direct-telemetry
#   --gait NAME (default rug_hstep, uneven course. Perception guide s7.3.2/s8.2.2 pairs even floors with the even slow/medium
#     gait and uneven ground with the high-step gait; was rug_general) --rough-gait NAME (default rug_hstep,
#     used for stairs/rubble). ROAM_GAIT= keeps the stand-up gait.
#   --sim only: --sim-seconds S  --sim-script "60:pause;75:resume;90:speed:fast"
# --max-speed V is the session ceiling: navigation is started (if not running) with --max-speed V
# --clearance ${ROAM_NAV_CLEARANCE:-0.0}, so the collision stop boundary is computed for V, and roaming
# Normal starts at 0.50 m/s or the session limit, whichever is lower. Fast can use the session maximum.
# The documented forward navigation limit is 1.0 m/s. Braking is validated up to 0.10 m/s.
# THE HANDHELD REMOTE / E-STOP IS THE EMERGENCY STOP. "roam.sh stop" and Ctrl+C are commanded stops.
set -eo pipefail
source "$(dirname "$0")/env.sh"
ROAM_PID_FILE="$LOG_ROOT/roam.pid"
CMD="${1:-help}"; [ $# -gt 0 ] && shift

roam_running() { [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; }

case "$CMD" in
  start)
    PHYS=0; SIM=0; NAV_EXTRA=(); PY_ARGS=(); MAXV="${ROAM_NAV_MAX_SPEED:-0.50}"; LIMIT_GIVEN=0; GAIT_GIVEN=0
    while [ $# -gt 0 ]; do
      a="$1"; shift
      case "$a" in
        --max-speed) MAXV="$1"; shift; continue ;;
        --speed-limit) LIMIT_GIVEN=1 ;;
        --gait) GAIT_GIVEN=1 ;;
        --physical) PHYS=1 ;;
        --sim) SIM=1 ;;
        --direct-telemetry) NAV_EXTRA=(--direct-telemetry) ;;
      esac
      PY_ARGS+=("$a")
    done
    [ $LIMIT_GIVEN -eq 0 ] && PY_ARGS+=(--speed-limit "$MAXV")
    GAIT_DEFAULT="${ROAM_GAIT-rug_hstep}"   # course of 0.10-0.15 m lips (2026-11-21); was rug_general, flat_medium
    [ $GAIT_GIVEN -eq 0 ] && [ -n "$GAIT_DEFAULT" ] && PY_ARGS+=(--gait "$GAIT_DEFAULT")
    if nav_running && [ $SIM -eq 0 ]; then
      RUNNING_MAXV=$(sed -n 's/^MAX_SPEED=//p' "$STATE_FILE")
      if [ -n "$RUNNING_MAXV" ] && [ "$RUNNING_MAXV" != "$MAXV" ]; then
        echo "NOTE: navigation is running with --max-speed $RUNNING_MAXV (stop boundary $(sed -n 's/^BOUNDARY=//p' "$STATE_FILE") m)," \
             "not $MAXV. Roaming refuses a limit above $RUNNING_MAXV; for a matching boundary:" \
             "operator/stop_navigation.sh, then start again."
      fi
    fi
    if roam_running; then echo "Roaming is already running (pid $(cat "$ROAM_PID_FILE")): operator/roam.sh status"; exit 3; fi
    if [ $SIM -eq 0 ]; then
      if [ $PHYS -eq 1 ]; then
        if nav_running && [ "$(nav_mode)" != physical ]; then
          echo "REFUSED: navigation is running in $(nav_mode) mode. operator/stop_navigation.sh first."; exit 3; fi
        if ! nav_running; then
          echo "Starting navigation (PHYSICAL) for roaming..."
          "$OPERATOR_DIR/start_navigation.sh" --physical "${NAV_EXTRA[@]}" --clearance "${ROAM_NAV_CLEARANCE:-0.0}" --max-speed "$MAXV" || exit 3
        fi
        echo "PHYSICAL ROAMING. Handheld remote / e-stop in your hand for the whole session."
      elif ! nav_running; then
        echo "Starting navigation (dry) for the roaming check..."
        "$OPERATOR_DIR/start_navigation.sh" "${NAV_EXTRA[@]}" --clearance "${ROAM_NAV_CLEARANCE:-0.0}" --max-speed "$MAXV" || exit 3
      fi
    fi
    python3 -m lite3_movement_tests.roam run "${PY_ARGS[@]}" 0<&0 &
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
  plan)
    case " $* " in *" --max-speed "*) ;; *) set -- --max-speed "${ROAM_NAV_MAX_SPEED:-0.50}" "$@" ;; esac
    case " $* " in *" --clearance "*) ;; *) set -- --clearance "${ROAM_NAV_CLEARANCE:-0.0}" "$@" ;; esac
    exec python3 -m lite3_movement_tests.roam plan "$@" ;;
  speed)
    [ $# -eq 1 ] || { echo "Usage: operator/roam.sh speed slow|normal|fast|up|down"; exit 2; }
    exec python3 -m lite3_movement_tests.roam speed "$1" ;;
  stairs)
    [ $# -eq 1 ] || { echo "Usage: operator/roam.sh stairs up|down (from PAUSED, robot squarely facing the stairs)"; exit 2; }
    exec python3 -m lite3_movement_tests.roam stairs "$1" ;;
  gait)
    [ $# -eq 1 ] || { echo "Usage: operator/roam.sh gait rug_general|rug_grip|rug_hstep|flat_slow|flat_medium|flat_fast (while paused)"; exit 2; }
    exec python3 -m lite3_movement_tests.roam gait "$1" ;;
  help|-h|--help)
    sed -n '2,27p' "$0" | sed 's/^# \{0,1\}//'
    exec python3 -m lite3_movement_tests.roam run --help ;;
  *) echo "Unknown command '$CMD' (start, plan, pause, resume, stop, speed, gait, stairs, status, modes, help)"; exit 2 ;;
esac
