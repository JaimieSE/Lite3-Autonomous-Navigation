#!/usr/bin/env bash
# Build the project packages into ~/lite3_ws/install (university ~/ros2_ws is not touched).
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/local_setup.bash"
# The robot image installs the native Livox message definitions in this overlay.
if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi
# --base-paths src: only this workspace's sources. Without it colcon also crawls any nested checkout
# (e.g. a clone at ~/lite3_ws/lite3_ws), finds every package twice and refuses to build, which left
# the install stale (M10 zero_delivery fix not installed, 2026-11-18).
colcon build --base-paths src --packages-select lite3_livox_cloud lite3_locomotion lite3_movement_tests
echo "Build done. New shells pick it up automatically (~/.bashrc sources ~/lite3_ws/install)."
