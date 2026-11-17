# Sourced by every operator script. Mirrors ~/.bashrc, but explicitly, so scripts also work
# from a non-interactive SSH session.
LITE3_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
UNIVERSITY_WS="$HOME/ros2_ws"
OPERATOR_DIR="$LITE3_WS/operator"
LOG_ROOT="$LITE3_WS/log/operator"
STATE_FILE="$LOG_ROOT/state.env"
TEST4_PID_FILE="$LOG_ROOT/test4.pid"
PHYSICAL_PID_FILE="$LOG_ROOT/physical.pid"
export LITE3_STATE_FILE="$STATE_FILE"

for f in /opt/ros/humble/setup.bash "$UNIVERSITY_WS/install/local_setup.bash" \
         "$LITE3_WS/install/local_setup.bash"; do
  if [ ! -f "$f" ]; then
    echo "ERROR: missing $f (run operator/build.sh for the lite3_ws part)" >&2
    return 1 2>/dev/null || exit 1
  fi
  source "$f"
done
export ROS_DOMAIN_ID=8          # group-8 domain; the robot-side system computer uses 64
unset ROS_LOCALHOST_ONLY
mkdir -p "$LOG_ROOT"

# Returns 0 if the navigation stack started by this workflow is alive.
nav_running() {
  [ -f "$STATE_FILE" ] || return 1
  local pgid; pgid=$(sed -n 's/^PGID=//p' "$STATE_FILE")
  [ -n "$pgid" ] && pgrep -g "$pgid" >/dev/null 2>&1
}

# Process groups belonging to this workflow (also finds them if state.env was lost).
workflow_pgids() {
  {
    [ -f "$STATE_FILE" ] && sed -n 's/^PGID=//p' "$STATE_FILE"
    ps -u "$(id -u)" -o pgid=,args= | awk '/ros2 launch lite3_locomotion navigation.launch.py|lib\/lite3_locomotion\/terrain_monitor/ && !/awk/ {print $1}'
  } | sort -u | while read -r g; do pgrep -g "$g" >/dev/null 2>&1 && echo "$g"; done
}

# dry | physical: how the running stack was started (empty if not running).
nav_mode() {
  nav_running || return 0
  local m; m=$(sed -n 's/^MODE=//p' "$STATE_FILE"); echo "${m:-dry}"
}
