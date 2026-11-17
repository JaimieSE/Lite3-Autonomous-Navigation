#!/usr/bin/env bash
# Supervised physical navigation tests M0-M9 (and STAND / SIT).
#   DRY (default): plan + PASS criteria + read-only live readiness; nothing is published.
#   --sim:         rehearse against the software stand-in (no robot).
#   --physical:    ACTUATES. Needs operator/start_navigation.sh --physical and a typed
#                  confirmation for every test. Ctrl+C = commanded stop + rest check.
# Examples:  operator/run_physical.sh --list-phases
#            operator/run_physical.sh --phases M0,M1,M2 --physical
#            operator/run_physical.sh --phases M4 --shadow          (dry: no motion)
#            operator/run_physical.sh --physical --phases M4 --profile operator/profiles/m4_first_session.yaml
set -eo pipefail
source "$(dirname "$0")/env.sh"
for arg in "$@"; do
  case "$arg" in --list-phases|-h|--help) exec python3 -m lite3_movement_tests.physical_suite "$@" ;; esac
done
if [ -f "$PHYSICAL_PID_FILE" ] && kill -0 "$(cat "$PHYSICAL_PID_FILE")" 2>/dev/null; then
  echo "Physical tests are already running (pid $(cat "$PHYSICAL_PID_FILE"))."; exit 1
fi
if [ -f "$LOG_ROOT/roam.pid" ] && kill -0 "$(cat "$LOG_ROOT/roam.pid")" 2>/dev/null && \
   grep -q '"mode": "physical"' "$LOG_ROOT/roam/status.json" 2>/dev/null; then
  echo "M10 roaming is running on the robot; stop it first (operator/roam.sh stop)."; exit 1
fi
if [ -f "$TEST4_PID_FILE" ] && kill -0 "$(cat "$TEST4_PID_FILE")" 2>/dev/null; then
  echo "Test 4 is running; finish or abort it first (operator/abort_test4.sh)."; exit 1
fi
if [[ " $* " == *" --physical "* ]]; then
  if [ "$(nav_mode)" != physical ]; then
    echo "REFUSED: navigation is $( nav_running && echo "running in $(nav_mode) mode" || echo "not running")."
    echo "Physical tests need: operator/stop_navigation.sh (if running), then operator/start_navigation.sh --physical"
    exit 3
  fi
  echo "PHYSICAL TESTS. Handheld remote / e-stop in your hand. Ctrl+C stops the robot (commanded stop)."
fi
python3 -m lite3_movement_tests.physical_suite "$@" 0<&0 &   # keep the terminal for confirmations
TEST_PID=$!
echo "$TEST_PID" > "$PHYSICAL_PID_FILE"
trap 'kill -INT "$TEST_PID" 2>/dev/null' INT TERM
set +e
wait "$TEST_PID"; CODE=$?
[ $CODE -gt 128 ] && { wait "$TEST_PID"; CODE=$?; }
set -e
rm -f "$PHYSICAL_PID_FILE"
exit $CODE
