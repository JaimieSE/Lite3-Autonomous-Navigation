#!/usr/bin/env bash
# Record what the terrain monitor sees right now (read-only, nothing moves): raw LiDAR, depth points, IMU,
# joints, odometry, TF and the terrain monitor's own grid/status. For offline obstacle diagnosis.
#   operator/capture_terrain.sh [label] [seconds]     e.g. operator/capture_terrain.sh roller 8
set -eo pipefail
source "$(dirname "$0")/env.sh"
LABEL="${1:-capture}"; SECS="${2:-8}"
OUT="$LOG_ROOT/captures/$(date +%Y%m%d-%H%M%S)-$LABEL"
mkdir -p "$(dirname "$OUT")"
echo "Recording ${SECS} s into $OUT (robot stays as it is)"
timeout -s INT "$SECS" ros2 bag record -o "$OUT" \
  /lite3/livox_points /lite3/depth_points /lite3/imu_data /lite3/joint_states /odom /tf /tf_static \
  /lite3_terrain/grid /lite3_terrain/status /lite3_terrain/hazards >/dev/null 2>&1 || true
ros2 bag info "$OUT" | sed -n '1,8p'
