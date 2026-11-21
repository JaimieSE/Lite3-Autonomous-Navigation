#!/usr/bin/env bash
# Run on the system host from the user-owned group8_lio deployment directory.
set -eo pipefail
cd "$(dirname "$0")"
exec 9>".mapper.lock"
flock -n 9 || { echo "This FAST-LIO deployment is already running"; exit 1; }
source /opt/ros/humble/setup.bash
source "$HOME/lite3_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=64
export OMP_NUM_THREADS=2
mapper=./fast_lio/lib/fast_lio/fastlio_mapping
config=./lite3_launcher/share/lite3_launcher/config/mid360.yaml
test -x "$mapper" && test -r "$config" || { echo "Missing mapper or Mid-360 calibration"; exit 1; }
test -x ./pointcloud_to_laserscan_node && test -r pointcloud_to_scan.yaml && test -r rgb_preview.py
pids=()
trap 'if ((${#pids[@]})); then kill -TERM "${pids[@]}" 2>/dev/null || true; wait || true; fi' EXIT
trap 'exit 0' INT TERM
./fast_lio/lib/fast_lio/fastlio_mapping --ros-args --params-file "$config" \
  -r __node:=group8_onboard_lio -r __ns:=/group8_lio \
  -r /Odometry:=/group8_lio/odometry -r /cloud_registered_body:=/group8_lio/cloud_registered_body \
  -r /cloud_registered:=/group8_lio/cloud_registered -r /cloud_effected:=/group8_lio/cloud_effected \
  -r /Laser_map:=/group8_lio/map -r /path:=/group8_lio/path &
pids+=("$!")
LD_LIBRARY_PATH="$PWD:${LD_LIBRARY_PATH:-}" ./pointcloud_to_laserscan_node --ros-args --params-file pointcloud_to_scan.yaml \
  -r cloud_in:=/group8_lio/cloud_registered_body -r scan:=/group8_lio/scan &
pids+=("$!")
python3 rgb_preview.py &
pids+=("$!")
/opt/ros/humble/lib/domain_bridge/domain_bridge outputs.yaml &
pids+=("$!")
wait -n "${pids[0]}" "${pids[1]}" "${pids[3]}"
