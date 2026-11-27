#!/usr/bin/env bash
# One command for a competition roam (operator 2026-11-24): readiness check, navigation, Pi mission reporter, RViz with
# every roam indicator, then roam with the unified decision (course grid memory, one evaluator, stuck ladder with Pi
# scene understanding, pauses that resume by themselves). Nothing moves until you type ARM and ROAM.
#
#   operator/competition_start.sh --physical      ARMED run: starts navigation (type ARM), then roam (type ROAM);
#                                                 after ROAM it puts the robot in navigation (auto) mode itself
#   operator/competition_start.sh                 dry: real sensors, robot still; RViz shows what roam would do
#   operator/competition_start.sh --sim           rehearsal against the software stand-in (no robot, no RViz)
#   operator/competition_start.sh status          roam status and the helpers this command started
#   operator/competition_start.sh stop            stop roam, the helpers, then navigation
#
# Options (before any roam option):
#   --missions FILE   local missions JSON for the reporter (default: its built-in booklet v6.4 list)
#   --no-reporter     do not start ~/live_mission/pi_mission_reporter.py (it decides M01-M05; it never moves the robot)
#   --arena A|B       scoring endpoint for the reporter: task_report_A or task_report_B (default)
#   --dry-report      keep the reporter in dry run on a --physical run (prints payloads, no POST)
#                     The reporter POSTs (--live) only on --physical runs and only with a valid group_id.txt;
#                     dry and --sim runs never POST.
#   --no-rviz         do not open RViz (operator/competition.rviz)
#   --no-scene-hold   do not start ~/live_mission/nav_hold.py (Pi scene checks then never stop the dog)
#   --no-capture      do not start the camera capture (NUC relay + Pi Claude half); pockets then get no reading:
#                     roam waits 12 s for an ack that never comes, then moves on
#   --no-centre-lane  walk the chosen goals directly (default: cell centre to cell centre on the 1 x 1 m grid)
# When roam ends, RViz, navigation and the mission reporter stay up ("stop" ends them; operator 2026-11-27).
# Every other option goes to roam (operator/roam.sh start), e.g. --max-minutes 30, --auto-resume 20,
# --weights "grid=2", --geofence 10. Speed is normal (regular) by default.
# THE HANDHELD REMOTE / E-STOP IS THE EMERGENCY STOP.
set -eo pipefail
source "$(dirname "$0")/env.sh"
PIDS_FILE="$LOG_ROOT/competition.pids"
ROAM_PID_FILE="$LOG_ROOT/roam.pid"
export LITE3_LIO_SOURCE="${LITE3_LIO_SOURCE:-local}"

helpers_stop() {
  timeout 10 ssh -o BatchMode=yes -o ConnectTimeout=5 group8pi '~/group8-capture/run.sh stop' 2>/dev/null || true
  [ -f "$PIDS_FILE" ] || return 0
  while read -r name pid; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      echo "   stopping $name (pid $pid)"
      kill -INT "$pid" 2>/dev/null || true
      for _ in $(seq 1 10); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
      kill -0 "$pid" 2>/dev/null && kill -TERM "$pid" 2>/dev/null || true
    fi
  done < "$PIDS_FILE"
  rm -f "$PIDS_FILE"
}

case "${1:-}" in
  stop)
    echo "== Competition stop"
    if [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; then
      "$OPERATOR_DIR/roam.sh" stop || true
      for _ in $(seq 1 40); do kill -0 "$(cat "$ROAM_PID_FILE" 2>/dev/null)" 2>/dev/null || break; sleep 0.5; done
    fi
    helpers_stop
    exec "$OPERATOR_DIR/stop_navigation.sh" ;;
  status)
    "$OPERATOR_DIR/roam.sh" status || true
    if [ -f "$PIDS_FILE" ]; then
      while read -r name pid; do
        if kill -0 "$pid" 2>/dev/null; then echo "$name: running (pid $pid)"; else echo "$name: not running"; fi
      done < "$PIDS_FILE"
    fi
    echo "navigation: $(nav_running && echo "running ($(nav_mode))" || echo 'not running')"
    exit 0 ;;
  -h|--help|help)
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
esac

MODE=dry; REPORTER=1; RVIZ=1; SCENE_HOLD=1; CAPTURE=1; CENTRE_LANE=1; ARENA=B; DRY_REPORT=0
MISSIONS=""
ROAM_ARGS=()
while [ $# -gt 0 ]; do
  a="$1"; shift
  case "$a" in
    --physical) MODE=physical; ROAM_ARGS+=(--physical --auto-mode on) ;;   # robot to navigation (auto) mode
    --sim) MODE=sim; ROAM_ARGS+=(--sim) ;;
    --dry) MODE=dry ;;
    --missions) MISSIONS="$1"; shift ;;
    --arena) ARENA="${1^^}"; shift
             case "$ARENA" in A|B) ;; *) echo "--arena must be A or B"; exit 2 ;; esac ;;
    --dry-report) DRY_REPORT=1 ;;
    --no-reporter|--no-pi-bridge) REPORTER=0 ;;   # --no-pi-bridge: the retired bridge's old flag
    --no-rviz) RVIZ=0 ;;
    --no-scene-hold) SCENE_HOLD=0 ;;
    --no-capture) CAPTURE=0 ;;
    --no-centre-lane) CENTRE_LANE=0 ;;
    --centre-lane) CENTRE_LANE=1 ;;
    *) ROAM_ARGS+=("$a") ;;
  esac
done
if [ -f "$ROAM_PID_FILE" ] && kill -0 "$(cat "$ROAM_PID_FILE")" 2>/dev/null; then
  echo "Roam is already running (pid $(cat "$ROAM_PID_FILE")): operator/competition_start.sh status | stop"; exit 3
fi
# Operator 2026-11-26: competition runs walk the way the 20 Nov run did (the run that covered the most meaningful
# ground: 7.2 m out, 74.5 m2 mapped, 16/17 goals, no turns on the spot): the sequence decision - hold the heading
# and walk on, else the smallest turn that opens, else a frontier, else 90-degree search turns. Kept from later:
# cell-centre walking (centre lanes), 90-degree turns, 0.5 rad/s turning. No look-arounds, pose looks or photo stops.
# Pass --decision unified for the newer one-evaluator search.
# Operator 2026-11-26 evening spec (replaces the line above's default): the grid decision - corridor cells walked
# centre to centre with exact 90-degree turns, rooms (dead ends) read from the front with 3 body-twist views, never
# entered, frontier only once the grid is done. --decision sequence / unified still select the others.
case " ${ROAM_ARGS[*]} " in *" --decision "*) ;; *) ROAM_ARGS+=(--decision grid) ;; esac
case " ${ROAM_ARGS[*]} " in *" --speed "*) ;; *) ROAM_ARGS+=(--speed normal) ;; esac
# Operator 2026-11-27: the terminal shows plain, important lines only (raw lines: <report dir>/terminal_full.log).
# Pass --terminal full for the old detailed output.
case " ${ROAM_ARGS[*]} " in *" --terminal "*) ;; *) ROAM_ARGS+=(--terminal plain) ;; esac
# Operator 2026-11-26: the Pi's stairs YOLO alone hands over to the AI-mode stair crossing (no RGB-D / VLM / terrain
# reconfirmation). Pass --stairs-on-sight or --defer-stairs to use the confirmed paths instead.
case " ${ROAM_ARGS[*]} " in *" --stairs-on-sight "*|*" --defer-stairs "*|*" --stairs-pi-only "*) ;;
  *) ROAM_ARGS+=(--stairs-pi-only) ;; esac
# Operator 2026-11-24: competition runs walk cell centre to cell centre (teammate's centre lanes, merged).
[ $CENTRE_LANE -eq 1 ] && ROAM_ARGS+=(--centre-lane)

RUN="$LOG_ROOT/competition-$(date +%Y%m%d-%H%M%S)-$MODE"
mkdir -p "$RUN"
echo "== Competition start ($MODE). Logs: $RUN"

if [ "$MODE" != sim ]; then
  # Operator 2026-11-26 (start-up time): the Pi check runs in the background while navigation is stopped / checked.
  PI_CHECK="$RUN/pi_check.txt"
  (timeout 8 ros2 topic info /group8pi/perception/detections 2>/dev/null || true) > "$PI_CHECK" &
  PI_CHECK_PID=$!
  # roam.sh refuses a physical run on a dry navigation stack (and the reverse is pointless): restart it
  if nav_running && [ "$(nav_mode)" != "$MODE" ]; then
    echo "== Navigation is running in $(nav_mode) mode; stopping it for a $MODE start"
    "$OPERATOR_DIR/stop_navigation.sh"
  fi
  # Readiness check once (2026-11-26): start_navigation.sh runs the same check before it launches, so here only
  # when navigation is already up in this mode and will be reused.
  if nav_running; then
    echo "== Readiness check (read-only)"
    "$OPERATOR_DIR/preflight.sh" --direct-telemetry 2>&1 | tee "$RUN/preflight.txt" | tail -15 || true
  else
    echo "== Readiness check: runs once, as navigation starts"
  fi
  # The previous run's reporter outlives roam (posting in manual): stop it before this run starts its own.
  if [ -f "$PIDS_FILE" ] && [ $REPORTER -eq 1 ]; then
    grep '^reporter ' "$PIDS_FILE" > "$PIDS_FILE.old" || true
    PIDS_SAVE="$PIDS_FILE"; PIDS_FILE="$PIDS_FILE.old"
    ( helpers_stop() { while read -r name pid; do [ -n "$pid" ] && kill -INT "$pid" 2>/dev/null && \
        echo "   stopping the previous $name (pid $pid)"; done < "$PIDS_FILE"; }; helpers_stop )
    PIDS_FILE="$PIDS_SAVE"; rm -f "$PIDS_FILE.old"
  fi
  : > "$PIDS_FILE"
  # Stairs and curtains come from the Pi (Orbbec on the Pi, same stair/curtain detector): warn if it is silent.
  wait "$PI_CHECK_PID" 2>/dev/null || true
  PI_PUBS=$(sed -n 's/^Publisher count: //p' "$PI_CHECK")
  if [ "${PI_PUBS:-0}" -gt 0 ]; then
    echo "== Pi perception: publishing (stairs/curtains, missions, scene hints)"
  else
    echo "   WARNING: no Pi perception publisher on /group8pi/perception/detections: no stair detection, no scene"
    echo "            hints. Check the Pi (systemctl --user status group8pi-perception) before typing ROAM."
  fi
  if [ $REPORTER -eq 1 ]; then
    REPORTER_PY="$HOME/live_mission/pi_mission_reporter.py"
    REPORTER_ARGS=(--report-log "$RUN/mission_reports.jsonl" --endpoint "$ARENA")
    GROUP_ID_FILE="$HOME/live_mission/group_id.txt"
    REPORT_KIND="DRY RUN: payloads only, no POST"
    if [ "$MODE" = physical ] && [ $DRY_REPORT -eq 0 ]; then
      if grep -qxE "group08_[A-Za-z0-9]{4}" "$GROUP_ID_FILE" 2>/dev/null; then
        REPORTER_ARGS+=(--live)
        REPORT_KIND="LIVE: POST to task_report_$ARENA as $(cat "$GROUP_ID_FILE")"
      else
        echo "   WARNING: $GROUP_ID_FILE is missing or not group08_XXXX: the reporter stays in DRY RUN"
      fi
    fi
    [ -n "$MISSIONS" ] && REPORTER_ARGS+=(--missions-file "$MISSIONS")
    if [ -f "$REPORTER_PY" ] && { [ -z "$MISSIONS" ] || [ -f "$MISSIONS" ]; }; then
      # Arena origin = the first map->base_link pose once navigation is up: start in the start cell facing +X.
      echo "== Pi mission reporter ($REPORT_KIND) -> $RUN/reporter.log"
      setsid python3 "$REPORTER_PY" "${REPORTER_ARGS[@]}" > "$RUN/reporter.log" 2>&1 < /dev/null &
      echo "reporter $!" >> "$PIDS_FILE"
    else
      echo "   NOTE: Pi mission reporter not started (missing $REPORTER_PY${MISSIONS:+ or $MISSIONS})"
    fi
  fi
  if [ $SCENE_HOLD -eq 1 ] && [ -f "$HOME/live_mission/nav_hold.py" ]; then
    # Pi scene check -> roam stops at a safe point, squares up to the grid, waits <= 6 s (never on stairs,
    # curtains or rollers). It only publishes /group8/navigation/hold_request; it never sends motion.
    echo "== Scene hold (nav_hold.py) -> $RUN/nav_hold.log"
    setsid python3 "$HOME/live_mission/nav_hold.py" > "$RUN/nav_hold.log" 2>&1 < /dev/null &
    echo "nav_hold $!" >> "$PIDS_FILE"
  fi
  if [ -f "$HOME/live_mission/stair_markers.py" ]; then
    # Display only: Pi stairs (obstacle YOLO + confirmed RGB-D) as labelled blocks in RViz (/group8pi/stair_markers).
    echo "== Stair labels for RViz (stair_markers.py) -> $RUN/stair_markers.log"
    setsid python3 "$HOME/live_mission/stair_markers.py" > "$RUN/stair_markers.log" 2>&1 < /dev/null &
    echo "stair_markers $!" >> "$PIDS_FILE"
  fi
  if [ $CAPTURE -eq 1 ] && [ -f "$HOME/live_mission/scene_capture.py" ]; then
    # Operator 2026-11-25 (item 7): dog stops in a pocket -> picture -> Claude -> stored for upload -> ack -> moves off.
    # NUC half (relay): attaches the robot camera frame to each request. Pi half (~/group8-capture/run.sh, the Claude
    # key stays on the Pi): asks Claude, stores the picture + answer (upload queue captures.jsonl), acks roam.
    echo "== Camera capture: NUC relay -> $RUN/scene_capture_relay.log; Pi: ~/group8-capture/store/$(basename "$RUN")"
    setsid python3 "$HOME/live_mission/scene_capture.py" --relay > "$RUN/scene_capture_relay.log" 2>&1 < /dev/null &
    echo "scene_capture_relay $!" >> "$PIDS_FILE"
    if ! timeout 20 ssh -o BatchMode=yes -o ConnectTimeout=5 group8pi "~/group8-capture/run.sh start $(basename "$RUN")" \
        2>&1 | tail -1; then
      echo "   WARNING: Pi capture not started: pockets get no Claude reading (roam waits 12 s, then moves on)"
    fi
  fi
  if [ $RVIZ -eq 1 ]; then
    if [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v rviz2 >/dev/null; then
      echo "== RViz (operator/competition.rviz): map, costmap, plan, roam grid/goal/alternatives/status line"
      setsid rviz2 -d "$OPERATOR_DIR/competition.rviz" > "$RUN/rviz.log" 2>&1 < /dev/null &
      echo "rviz $!" >> "$PIDS_FILE"
    else
      echo "   NOTE: no display here; open RViz yourself: rviz2 -d $OPERATOR_DIR/competition.rviz"
    fi
  fi
fi

on_exit() {
  code=$?
  echo "== Roam ended (exit $code). RViz, navigation and the mission reporter stay up."
  if [ -f "$PIDS_FILE" ]; then
    # Operator 2026-11-27: a manual scene fallback (scene_check.sh --fallback on) still running needs the mission
    # reporter - roam ending (e.g. remote takeover) must not stop it. It stops with the fallback or with "stop".
    # Operator 2026-11-27: "It should be posting at all times, even in manual" - the reporter is never stopped
    # with roam (manual driving + scene_check.sh --fallback keeps posting); only "competition_start.sh stop" ends it.
    KEEP='^rviz \|^reporter '
    echo "   The mission reporter keeps running (posting continues in manual); 'stop' ends it."
    grep "$KEEP" "$PIDS_FILE" > "$PIDS_FILE.stay" || true
    grep -v "$KEEP" "$PIDS_FILE" > "$PIDS_FILE.keep" || true      # (reporter and nav_hold stop with roam)
    mv "$PIDS_FILE.keep" "$PIDS_FILE"
    helpers_stop
    mv "$PIDS_FILE.stay" "$PIDS_FILE"                              # "stop" ends these later
  fi
  echo "   Reports: $(ls -dt "$LOG_ROOT"/run-*/roam/*/ 2>/dev/null | head -1) (report.md, mission_events.json)"
  [ "$MODE" != sim ] && echo "   Mission reports (dry): $RUN/reporter.log, $RUN/mission_reports.jsonl   End everything: operator/competition_start.sh stop"
  return 0
}
trap on_exit EXIT

echo "== Roam: ${ROAM_ARGS[*]}"
"$OPERATOR_DIR/roam.sh" start "${ROAM_ARGS[@]}"
