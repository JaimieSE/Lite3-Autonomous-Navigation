#!/usr/bin/env bash
# lite3_auto (operator 2026-11-27): put the robot in navigation (auto) mode.
# While roam runs, roam sends it (move + navigation mode + gait): a direct /simple_cmd would be a second command
# source and roam would safety-stop. Without roam, the old one-shot /simple_cmd publish.
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "$DIR/../log/operator/roam.pid" ] && kill -0 "$(cat "$DIR/../log/operator/roam.pid")" 2>/dev/null; then
  exec "$DIR/roam.sh" automode
fi
exec ros2 topic pub /simple_cmd transfer_interfaces/msg/MotionSimpleCMD "{cmd_code: 0x21010C03, type: 0}" --once
