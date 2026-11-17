#!/usr/bin/env bash
# Build the project packages into ~/lite3_ws/install (university ~/ros2_ws is not touched).
set -eo pipefail
cd "$(dirname "$0")/.."
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/local_setup.bash"
colcon build --packages-select lite3_locomotion lite3_movement_tests
echo "Build done. New shells pick it up automatically (~/.bashrc sources ~/lite3_ws/install)."
