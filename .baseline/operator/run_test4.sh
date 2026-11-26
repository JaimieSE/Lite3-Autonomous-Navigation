#!/usr/bin/env bash
# Test 4: five-minute non-actuating navigation test with real obstacles. Ctrl+C aborts.
set -eo pipefail
source "$(dirname "$0")/env.sh"
SOAK="$LITE3_WS/src/lite3_locomotion/tools/nav_dry_soak.py"
for arg in "$@"; do
  case "$arg" in
    --physical) exec python3 "$SOAK" "$@" ;;           # explains the physical counterparts; exit 2
    --list-phases|-h|--help) exec python3 "$SOAK" "$@" ;;
  esac
done
if [ "$(nav_mode)" = physical ]; then
  echo "Navigation is running in PHYSICAL mode. Test 4 is a dry test; stop and restart dry:"
  echo "  operator/stop_navigation.sh && operator/start_navigation.sh"
  exit 1
fi
if ! nav_running; then
  echo "Navigation is not running. Start it first: operator/start_navigation.sh"
  exit 1
fi
if [ -f "$TEST4_PID_FILE" ] && kill -0 "$(cat "$TEST4_PID_FILE")" 2>/dev/null; then
  echo "Test 4 is already running (pid $(cat "$TEST4_PID_FILE"))."
  exit 1
fi
RUN_DIR=$(sed -n 's/^RUN_DIR=//p' "$STATE_FILE")
echo "Verifying non-actuation before the test..."
set +e
python3 "$OPERATOR_DIR/lite3ops.py" status --json "$RUN_DIR/test4-precheck.json"
PRE=$?
set -e
if [ $PRE -eq 1 ]; then
  echo "NOT STARTING TEST 4: status check failed (see above)."
  exit 1
fi
echo
echo "Test 4 starts in 5 s. Ctrl+C at any time aborts it (goal cancelled, partial report saved)."
echo "Options: --phases P3 (or P2,P3)  --manual (Enter starts each phase)  --list-phases  --stop-distance 0.40"
sleep 5
python3 "$SOAK" --report-dir "$RUN_DIR/test4" "$@" 0<&0 &   # keep terminal stdin for --manual
TEST_PID=$!
echo "$TEST_PID" > "$TEST4_PID_FILE"
trap 'kill -INT "$TEST_PID" 2>/dev/null' INT TERM
set +e
wait "$TEST_PID"; CODE=$?
[ $CODE -gt 128 ] && { wait "$TEST_PID"; CODE=$?; }
set -e
rm -f "$TEST4_PID_FILE"
echo "Results are under $RUN_DIR/test4/. Navigation is still running (dry); stop it with operator/stop_navigation.sh"
exit $CODE
