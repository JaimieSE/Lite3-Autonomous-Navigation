#!/usr/bin/env bash
# Start perception + SLAM + Nav2 + terrain monitor + twist_mux.
# Usage: operator/start_navigation.sh [--accept-degraded]   DRY (default): no robot commands
#        Tests (M0-M9, roaming + its watchdog) read robot telemetry at its source in domain 64 (default,
#        --direct-telemetry is accepted and changes nothing). --bridge-telemetry reads the university
#        domain-8 bridge instead: measured 2026-11-19 (stationary, 300 s) at 89 Hz with 29-31 gaps of
#        0.25-0.50 s and 1-message seconds, vs 199.5 Hz / max gap 0.12 s at the source - enough to trip
#        M2's 0.25 s freshness stop and the watchdog's 20 Hz readiness.
#        --stop-layers docs|full (default docs, operator 2026-11-20): docs = obstacle avoidance by Nav2
#            planning only, as the vendor navigation (Perception Development Guide s8.2); the remote STOP
#            button is the emergency stop. full = also the collision_monitor stop zone and the terrain
#            proximity lock (use for the supervised M-series tests).
#        --clearance M (default 0.15): obstacle clearance the collision_monitor stop must leave
#        --max-speed V (default 0.10): fastest speed that boundary must cover (<= 1.0 m/s; only <= 0.10 is
#            validated. Above 0.10 the boundary grows fast: braking distance is taken as quadratic in speed)
#        operator/start_navigation.sh --physical             ARMED: twist_mux -> /cmd_vel -> robot,
#            behind the operator dead-man lock (robot stays locked until a supervised
#            operator/run_physical.sh --physical test holds it). Asks you to type ARM.
set -eo pipefail
source "$(dirname "$0")/env.sh"
ACCEPT_DEGRADED=0
MODE=dry
TELEMETRY=direct
CLEARANCE=0.15
MAX_SPEED=0.10
STOP_LAYERS=docs
PREFLIGHT_ARGS=(--direct-telemetry)
LAUNCH_ARGS=()
while [ $# -gt 0 ]; do
  arg="$1"; shift
  case "$arg" in
    --clearance) CLEARANCE="$1"; shift ;;
    --max-speed) MAX_SPEED="$1"; shift ;;
    --stop-layers) STOP_LAYERS="$1"; shift ;;
    --full-stops) STOP_LAYERS=full ;;
    --accept-degraded) ACCEPT_DEGRADED=1 ;;
    --physical) MODE=physical ;;
    --direct-telemetry) TELEMETRY=direct; PREFLIGHT_ARGS=(--direct-telemetry) ;;
    --bridge-telemetry) TELEMETRY=bridge; PREFLIGHT_ARGS=() ;;
    *) echo "Unknown option: $arg (see the header of this script)"; exit 2 ;;
  esac
done
case "$STOP_LAYERS" in docs|full) ;; *) echo "--stop-layers must be docs or full"; exit 2 ;; esac
CALIBRATION="$LOG_ROOT/calibration.json"
if [ "$MODE" = physical ]; then
  if [ $ACCEPT_DEGRADED -eq 1 ]; then
    echo "REFUSED: --accept-degraded is not allowed with --physical (degraded LiDAR = unsafe)."; exit 2
  fi
  if [ ! -t 0 ]; then echo "REFUSED: --physical needs an interactive terminal (typed ARM)."; exit 2; fi
  echo "PHYSICAL MODE: twist_mux output will be the robot's /cmd_vel."
  echo "  The robot stays LOCKED (dead-man) until you run a supervised test with"
  echo "  operator/run_physical.sh --physical, which asks for confirmation per test."
  echo "  Keep the handheld remote / e-stop in reach from now on."
  read -r -p "Type ARM to start navigation in physical mode (anything else cancels): " ANSWER
  if [ "$ANSWER" != "ARM" ]; then echo "Not armed. Nothing was started."; exit 1; fi
  LAUNCH_ARGS=(mux_output:=/cmd_vel allow_robot_motion:=true)
fi

if nav_running; then
  echo "Navigation is already running in $(nav_mode) mode (started $(sed -n 's/^STARTED=//p' "$STATE_FILE"))."
  [ "$(nav_mode)" != "$MODE" ] && echo "To switch to $MODE mode: operator/stop_navigation.sh, then start again."
  echo "Check it with operator/status.sh, or stop it with operator/stop_navigation.sh."
  exit 0
fi
if [ -n "$(workflow_pgids)" ]; then
  echo "ERROR: navigation processes from an earlier run are still alive (PGIDs: $(workflow_pgids | tr '\n' ' '))."
  echo "Run operator/stop_navigation.sh first."
  exit 1
fi

RUN_DIR="$LOG_ROOT/run-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN_DIR"
echo "== 1/3 Preflight"
set +e
python3 "$OPERATOR_DIR/lite3ops.py" preflight "${PREFLIGHT_ARGS[@]}" --json "$RUN_DIR/preflight.json"
PRE=$?
set -e
if grep -q '"item": "navigation stack"' "$RUN_DIR/preflight.json" && \
   grep -q 'already running' "$RUN_DIR/preflight.json"; then
  echo "ERROR: navigation nodes already exist in domain 8 but were not started by this workflow"
  echo "(another terminal or user). Not starting a duplicate. See operator/README.md 'Duplicate nodes'."
  exit 1
fi
if [ $PRE -eq 1 ]; then
  echo "NOT STARTING: preflight failed. Fix the FAIL items above (see operator/README.md Troubleshooting)."
  exit 1
fi
if [ $PRE -eq 2 ] && [ $ACCEPT_DEGRADED -eq 0 ]; then
  echo "NOT STARTING: preflight is DEGRADED (warnings above). Test results would be unreliable."
  echo "Fix the warnings, or re-run with --accept-degraded if you only want to inspect the stack."
  exit 1
fi

# docs: the stop polygon never triggers, so it is only sized (for 0.10 m/s) to stay readable; the speed
# ceiling MAX_SPEED is then not limited by a braking distance (manufacturer limit 1.0 m/s still applies).
CM_SPEED="$MAX_SPEED"
[ "$STOP_LAYERS" = docs ] && CM_SPEED=0.10
BOUNDARY_JSON=$(python3 -m lite3_locomotion.collision_profile --clearance "$CLEARANCE" \
  --max-speed "$CM_SPEED" --calibration "$CALIBRATION")
BOUNDARY=$(echo "$BOUNDARY_JSON" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("boundary_m",""))')
if [ -z "$BOUNDARY" ]; then
  echo "REFUSED: cannot compute the stop boundary: $BOUNDARY_JSON"; exit 2
fi
APPROACH_S=$(echo "$BOUNDARY_JSON" | python3 -c 'import json,sys; d=json.load(sys.stdin); print(d.get("approach_time_s",""))')
if [ "$STOP_LAYERS" = docs ]; then
  :
elif [ -n "$APPROACH_S" ]; then
  echo "Collision stop boundary: $BOUNDARY m from the nose (clearance $CLEARANCE m at <= 0.10 m/s)"
  echo "Collision APPROACH: full speed unless a collision along the current command is predicted within"
  echo "  $APPROACH_S s (then slowed so it reaches the stop zone at <= 0.10 m/s). Braking above 0.10 m/s is assumed."
else
  echo "Collision stop boundary: $BOUNDARY m from the nose (clearance $CLEARANCE m at <= $MAX_SPEED m/s)"
fi
echo "  $BOUNDARY_JSON"
LAUNCH_ARGS+=(clearance:="$CLEARANCE" max_speed:="$CM_SPEED" calibration:="$CALIBRATION" stop_layers:="$STOP_LAYERS")
if [ "$STOP_LAYERS" = docs ]; then
  echo "Stop layers: DOCS - Nav2 avoids obstacles (costmaps + planners) as in the vendor navigation;"
  echo "  no collision_monitor stop zone, no terrain proximity lock. Remote STOP = emergency stop."
  APPROACH_S=
fi
if [ "$MODE" = physical ]; then
  echo "== 2/3 Launching (PHYSICAL: twist_mux -> /cmd_vel, dead-man locked). Log: $RUN_DIR/launch.log"
else
  echo "== 2/3 Launching (dry: twist_mux -> /lite3_locomotion/dry_cmd_vel). Log: $RUN_DIR/launch.log"
fi
export ROS_LOG_DIR="$RUN_DIR/ros_logs"
setsid bash -c 'echo $$ > "$1"; shift; exec ros2 launch lite3_locomotion navigation.launch.py "$@"' _ \
  "$RUN_DIR/pgid" "${LAUNCH_ARGS[@]}" > "$RUN_DIR/launch.log" 2>&1 < /dev/null &
sleep 1
PGID=$(cat "$RUN_DIR/pgid")
printf 'PGID=%s\nRUN_DIR=%s\nSTARTED=%s\nMODE=%s\nTELEMETRY=%s\nCLEARANCE=%s\nMAX_SPEED=%s\nBOUNDARY=%s\nAPPROACH_S=%s\nSTOP_LAYERS=%s\n' \
  "$PGID" "$RUN_DIR" "$(date '+%F %T')" "$MODE" "$TELEMETRY" "$CLEARANCE" "$MAX_SPEED" "$BOUNDARY" \
  "$APPROACH_S" "$STOP_LAYERS" > "$STATE_FILE"

if [ "$MODE" = physical ]; then
  echo "== 3/3 Waiting for readiness and verifying twist_mux is the only, dead-man-locked command path"
else
  echo "== 3/3 Waiting for readiness and verifying the robot command path is NOT driven"
fi
set +e
python3 "$OPERATOR_DIR/lite3ops.py" wait-ready --mode "$MODE" --timeout 120 --json "$RUN_DIR/ready.json"
READY=$?
set -e
if [ $READY -eq 1 ]; then
  echo
  echo "STARTUP FAILED. Last launch log lines:"
  grep -E "ERROR|FATAL|exception" "$RUN_DIR/launch.log" | tail -n 8 || true
  echo "Stopping the partial stack..."
  "$OPERATOR_DIR/stop_navigation.sh"
  echo "Recovery: run operator/preflight.sh, fix FAIL items, then start again. Full log: $RUN_DIR/launch.log"
  exit 1
fi
echo
if [ "$MODE" = physical ]; then
  echo "NAVIGATION READY (PHYSICAL, dead-man LOCKED until a supervised test). Logs: $RUN_DIR"
  echo "Next: operator/run_physical.sh --physical --phases M0    Stop: operator/stop_navigation.sh"
else
  echo "NAVIGATION READY (dry, non-actuating). Logs: $RUN_DIR"
  echo "Next: operator/run_test4.sh    Status: operator/status.sh    Stop: operator/stop_navigation.sh"
fi
