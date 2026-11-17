#!/usr/bin/env bash
# Walk to a point ahead with Nav2; Nav2's planner/controller go around obstacles on the way.
# The collision_monitor stops the robot if an obstacle gets inside its stop boundary anyway.
# Usage: operator/walk.sh [--distance 2.0] [--left 0.0] [--speed 0.05] [--physical]
#   without --physical: plan + read-only readiness (nothing moves)
#   with --physical:    needs operator/start_navigation.sh --physical; you type WALK to start.
set -eo pipefail
DIST=2.0; LEFT=0.0; SPEED=0.05; EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --distance) DIST="$2"; shift 2 ;;
    --left) LEFT="$2"; shift 2 ;;
    --speed) SPEED="$2"; shift 2 ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done
exec "$(dirname "$0")/run_physical.sh" --phases WALK --goal-distance "$DIST" --goal-left "$LEFT" \
  --max-speed "$SPEED" "${EXTRA[@]}"
