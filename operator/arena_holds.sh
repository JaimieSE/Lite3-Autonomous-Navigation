#!/usr/bin/env bash
# Test command (operator 2026-11-26): the competition run plus --arena-holds, to compare both in a real arena run.
# operator/competition_start.sh is NOT changed and does not enable any of this.
#
# --arena-holds (roam):
#   - grid steps: face the next cell first (one in-place turn onto the grid direction, two quarter-turns for a cell
#     behind if there is turning room), check the heading, then walk; a step turn that stops early or a walk that
#     leaves the lane holds 2 s and reconsiders (that cell is left alone for 20 s; no failed destination)
#   - after the stairs have fully finished and SLAM has resumed: wait for a fresh map, costmap and a steady pose,
#     then correct to the cell centre / grid direction inside the same cell, clear of the stairs; if that cannot be
#     verified, pause and log why
#   - posture look-around: each view's heading is checked and logged
#   The stair detection, approach, AI-mode crossing and SLAM pause/resume are unchanged.
#
#   operator/arena_holds.sh --physical        same options as competition_start.sh
#   operator/arena_holds.sh status | stop     passed straight through
set -eo pipefail
case "${1:-}" in
  stop|status) exec "$(dirname "$0")/competition_start.sh" "$@" ;;
esac
exec "$(dirname "$0")/competition_start.sh" "$@" --arena-holds
