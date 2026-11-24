# One goal-selection system for competition roam: plan

Status (2026-11-24): **stages 1–5 implemented and simulator-tested**. Not yet tried on the robot.
The plan below is kept as written; see "Implemented" right here for what changed against it.

## Implemented (2026-11-24)
- **One decision.** `roam ... --decision unified` is now the command-line default. Every source proposes:
  - scan-walk straight-on and corridors;
  - frontier and patrol;
  - "come back" cells from the course grid;
  - scene hints.

  `goal_selection.py` merges and scores the proposals and logs a `decision` event with the breakdown and the
  next 4 alternatives. Roam is still the only thing that moves the robot. `--decision sequence` keeps the old
  fixed order.
- **Grid = memory.** `course_grid.py` aligns with the start pose by default and keeps three separate facts per
  cell: mapped, visited (walked) and seen (by the camera). It also keeps connection states and failure memory,
  and offers "come back" points (off-centre, at least 0.5 m of free floor, checked against the costmap). The
  grid never sends a goal by itself.
- **Reselection with hysteresis.** A walk goal is replaced only if:
  - it has become invalid; or
  - the new goal scores at least 1.3 × the current one + 1.0, the current goal has run at least 6 s, the robot
    is more than 1 m from it, and there has been no switch in the last 10 s.

  Stairs, curtains, passages, inspections, backtracks and scene walks are never replaced.
- **Stuck ladder**, triggered by any of: no worthwhile goal, 5 failed walks in a row, a turn that can't turn,
  or under 0.5 m moved and under 0.5 m² newly mapped or seen in 60 s. The steps are:
  1. Look around.
  2. Ask the Pi for a scene.
  3. Head back to an unfinished cell.
  4. Clear the costmaps.
  5. Backtrack.
  6. Pause.
- **Pauses resume by themselves** (`--auto-resume 15`, doubling up to 60 s). `roam.sh pause` still waits for you,
  and safety stops still end the session.
- **Mission events** are written to `mission_events.json` in each roam report folder.
- **RViz** on `/exploration/status_markers`:
  - grid squares: orange "come back", green seen, grey mapped, blue outline for walked cells;
  - a red X on blocked connections;
  - grey spheres with scores for the alternatives;
  - a status line over the dog: `ROAMING`, `GOAL: …`, `LOOKING`, `STUCK: …`, `PAUSED: resume 12s`,
    `HOLD`, `STEP CROSSING`, `STAIRS`, `CURTAIN CROSSING`.
- **One command: `operator/competition_start.sh`** (`--help` lists everything). It runs:
  1. the read-only readiness check;
  2. a navigation restart if the running stack is in the other mode;
  3. the Pi mission bridge (`~/live_mission/pi_bridge.sh --dry-run`; decisions only, never moves the robot);
  4. RViz with `operator/competition.rviz`;
  5. roam with `--decision unified --speed normal`, which starts navigation and asks for `ARM`, then `ROAM`.

  When roam ends, the bridge stops; RViz and navigation stay up. `competition_start.sh stop` ends everything,
  and `competition_start.sh status` shows the state. Logs go to `log/operator/competition-<time>-<mode>/`.
- **RViz layout `competition.rviz`:** map, global costmap, `/scan`, robot pose, Nav2 plan, the roam status
  markers (grid, goal, alternatives, status line) and the `live_mission` candidate pins.
- **Not yet done:**
  - plan-only Nav2 reachability (Nav2 still plans when the goal is sent);
  - saved-map capture.

### Pi scene interface (for whoever maintains the Pi code)
When stuck, roam publishes `/exploration/scene_request`:

```
{"request_id", "reason", "map_session", "pose": {"x", "y", "yaw"}, "stamp"}
```

It then waits 20 s (`--scene-timeout`) for the next `/group8pi/perception/detections` or
`/group8pi/perception/scenes` messages. It uses:
- `stair_curtain.obstacle` = `curtain` | `stairs` with a usable reason, and optional `heading_deg` and
  `distance_m`;
- scene fields `curtains[{image_region}]`, `stairs{observed, image_region}`, and `openings[{kind:
  gap|passage|curtain|stairs, image_region | bearing_deg}]`.

Image regions left, centre and right mean +25°, 0° and −25°. The Pi doesn't answer requests yet, so roam uses
whatever the Pi publishes next. Today's Pi scene messages carry only the room type, so in practice hints come
from `stair_curtain`. Limits: 6 requests per run, at least 60 s apart (`--scene-max`, `--scene-interval`).

### How to start the tests
```bash
cd ~/lite3_ws && source operator/env.sh && cd src/lite3_movement_tests
python3 -m pytest -q -p no:cacheprovider test/test_course_grid.py test/test_goal_selection.py test/test_scene_hints.py test/test_competition_scenarios.py
```
```bash
cd ~/lite3_ws && operator/competition_start.sh --sim --sim-seconds 300
```
```bash
cd ~/lite3_ws && operator/competition_start.sh
```
```bash
cd ~/lite3_ws && operator/competition_start.sh --physical
```
- The first command runs the unit and scenario tests on the simulator (about 3 minutes).
- `--sim` is a whole-loop rehearsal against the software stand-in. Its report is under the `sim` folder.
- With no flag (dry), the real sensors run with the robot still. RViz shows the grid, the alternatives and the
  status line on `/exploration/status_markers`.
- `--physical` runs on the robot, only on your instruction. Type `ARM`, then `ROAM`, and keep the remote STOP in
  hand.

## 1. Audit: what exists today

| Piece | Where | What it does | Verdict |
|---|---|---|---|
| Roam session (single movement owner) | `lite3_movement_tests/roam.py` `RoamSession` | Nav2 goals, cancel-with-ack (`cancel_request` waits for a terminal result), speed modes, holds, stairs/step crossing, drift nudge, advisory, pause/stop, watchdog, report | **Keep as the only movement owner** |
| Scan-walk | `roam.py` `ScanWalk` (`straight_ahead`, `choose`, `ray`, `novelty`) | Rays every few degrees, smallest turn into a corridor, walked-path novelty | Reuse; it returns **one** goal today, so it needs to return all its candidates |
| Frontier / patrol | `lite3_locomotion/frontiers.py`, `roam.py` `Explorer` | Frontier clusters plus "white spot" patrol, used as Nav2 goals | Reuse, return a list |
| Camera coverage | `coverage.py` `CameraCoverage` | 10 cm grid of what the camera has seen (80°, 3 m, ≤ 0.5 rad/s) | Reuse as the "visually inspected" layer |
| Exploration record | `exploration_record.py` | Destination and reason, breadcrumbs, junctions/branches, failure classes (temporary vs persistent, bounded retries), inspections, map-session reset, MarkerArray builder with tags | **Becomes the shared decision state** |
| Look-around, passage, backtrack, inspections | `roam.py` `_plan_look`, `_find_passage`, `_backtrack`, `_inspection_tick` | Run one after another in a fixed order in `_scan_walk`'s goal-is-None branch | Turn them into candidate generators instead of a fixed order |
| Grid search (time-limited exclusions) | `~/grid_roam/run_grid.py`, `preview.py` | Subclasses `RoamSession`; when idle, **replaces** roam's choice with a walk to an adjacent 1 m cell | Extract the logic, retire the wrapper |
| Grid search (Nav2 plan check) | `~/grid_roam_nav2/run_grid.py`, `preview.py` | Same, plus `PlanOnly` (ComputePathToPose without driving), off-centre waypoints (±0.12 m), three-lane crossing check between cells | Extract `Explorer.crossing/corridor` and `PlanOnly` |
| Mission recorder | `~/live_mission` `run_live.sh --publish-mission-events` | Publishes `/mission/events`; roam only logs them | Input for mission candidates |
| Pi perception bridge | `~/live_mission/pi_bridge.py`, `pi_mission.py` | Decides M01–M05 from Pi detections/scenes (dry), writes `status.json` | Source of "which missions are still open" |
| Scene understanding | `live_perception.py --scene-understanding` (`SceneScoring`), Pi `PI_SCENE_MODEL`, `~/scene_understanding` (do not touch) | Rate-limited room classification, **driven by detections only** | No way to ask "what is in front of me?" on demand |
| Stair/curtain advisory | `~/stair_curtain` → `/stair_curtain/advisory` | Roam already acts on it (curtain crossing, stairs routine) | Keep as a traversal handoff |

### Problems found in the grid wrappers (evidence: runs 2026-11-23 20:13–21:29)
- **Two strategies, one at a time.** When `goal is None` the wrapper's `_scan_walk` takes over before roam's own branch runs. Camera coverage, breadcrumbs, the record, the overlay, inspections, look-arounds and passages then **stop updating**, because all of that runs at the top of the base `_scan_walk`, which is skipped.
- **One cell at a time.** It proposes only the four adjacent cells, so it can't backtrack to a distant unfinished branch. The 21:29 runs covered 0.1, 1.6 and 3.9 m in 38–122 s, ending with 2–3 failures each and operator stops.
- **Failure memory.** The `nav2` version clears every exclusion whenever a new cell is visited, so retries are unbounded. The first version forgets after 30 s, without distinguishing temporary from persistent failures.
- **Private hooks.** It uses `robot._nodes`, `robot.wiring` and `run_session.__code__` rebinding, so it breaks silently when roam changes.
- **Manual alignment.** `--start-centre` and `--heading-deg` have to be typed for every course.
- **Strict "visited" test.** A cell counts as visited only when the robot centre gets within 0.15 m of the waypoint, so a reachable cell with a blocked centre is never done.

### Missing interfaces (block parts of the plan, see §8)
- Nothing publishes `/exploration/inspection_requests` or `/exploration/camera_confirmations`. Mission candidates and INSPECTED states work in tests only.
- Scene understanding can't be triggered on demand. There's no request/answer topic.
- There are no saved maps: run folders hold reports, not `/map` snapshots.

## 2. Target architecture

```
 inputs ─────────────────────────────────────────────────────────────────────────────┐
 /map, global costmap, terrain, pose, CameraCoverage, /mission/events, inspection     │
 requests, camera confirmations, advisory, scene hints                                 │
                                                                                       ▼
 ExplorationRecord (shared state, per map session)  +  CourseGrid (memory layer)  ──► generators ──► one evaluator ──► RoamSession (only mover)
        ▲                                                                                  │                │
        └──────────────── outcomes (reached / failed / confirmed) ◄────────────────────────┘                │
 build_markers(record, grid, decision) ──► /exploration/status_markers (RViz) ◄────────────────────────────┘
```

### 2.1 Course grid = memory, not a driver (operator 2026-11-24)
The grid never sends a goal by itself. It remembers, per cell, "this area still looks unexplored", and that memory pulls the robot back there as soon as nothing better is close.

`course_grid.py` (new, pure):
- **Alignment** is automatic: origin = start pose, +X = start heading. `--grid-origin X Y` and `--grid-heading-deg D` override it, and `--grid-cell` sets the cell size (default 1.0 m). Nothing about the course is hard-coded.
- **Per cell**, three distinct facts:
  - `mapped`: fraction of the cell's SLAM cells that are known.
  - `visited`: a breadcrumb lies inside the cell.
  - `seen`: fraction of the cell's free floor in `CameraCoverage`.
  
  Linked inspections are stored as ids from the record.
- **Connections** to the 4 neighbours: `open`, `unknown` or `blocked`. A blocked connection is either `temporary` (retry after 10 s, at most 3 times) or `persistent` (avoid for 120 s, at most 2 times). This reuses `ExplorationRecord.classify_failure` and its limits, and replaces the wrapper's ad-hoc sets. The crossing test is the three-lane `crossing()` from `~/grid_roam_nav2/preview.py`, copied with attribution. The teammate's folder isn't changed.
- **Unfinished cell**: mapped free floor ≥ 0.2 m², and either seen < 70 % or a connection still `unknown`. That's the "let's get back there" flag.
- **Session**: the grid is stored with `record.session` and cleared when the record detects a map reset.

### 2.2 Candidate generators (all return `Candidate` lists, none move the robot)
`goal_selection.py` (new, pure) defines:

```
Candidate(x, y, yaw, action='walk'|'turn'|'look', sources=[...], reasons=[...], terms={}, route='straight'|'nav2')
```

| Source | From | Note |
|---|---|---|
| `straight_on` / `corridor` | `ScanWalk.candidates()` (refactor of `choose` and `straight_ahead`: same rays, returns every open heading with its free length and novelty) | |
| `frontier` / `patrol` | `frontiers.select_frontiers` list (not only the selected one) | |
| `viewpoint` | New: for the 3 largest unseen-floor patches, a free pose 1–2.5 m away facing them | Off-centre poses allowed |
| `look` | Existing `_plan_look` sides, as turn candidates | |
| `inspection` | Every valid record inspection, not only `next_inspection` | Accepted even inside an already visited cell |
| `grid_return` | For each unfinished cell, the best free point in it or on its edge (`crossing` offsets, then free cells nearest the centre) | Used for the blocked centre and the distant branch |
| `backtrack` | Breadcrumb route toward the nearest unfinished cell or junction branch | Replaces the blind pop-the-trail backtrack |
| `scene_hint` | Answer to a stuck-time scene request (§2.5): curtain, gap or stairs at a bearing | |

Traversal routines (stairs, step crossing, curtain crossing, drift nudge) stay **handoffs** inside roam. They aren't candidates.

### 2.3 One evaluator
1. **Merge** candidates within 0.5 m and 45° of each other. Keep every source and reason. Take the **max** of each gain term (no double counting), plus a small `agreement` bonus per extra source.
2. **Filter**, cheapest first:
   - inside the geofence;
   - map and costmap fresh;
   - endpoint free on SLAM and below the inscribed cost (reuse `_reachable`);
   - not in a record failure window, and not across a persistent blocked connection;
   - straight-line corridor open (`corridor()`), **or** Nav2 `ComputePathToPose` succeeds. That call is plan-only, using `PlanOnly` adapted from `grid_roam_nav2`, async, cached for 3 s, and run only for the top 3 candidates that fail the straight test.
3. **Score**, with every weight configurable (`--weights "info=1,camera=0.8,..."` or a JSON file). Each decision logs the full breakdown for the chosen candidate and the next 4.

```
score =  w_info    * min(unknown_m2_revealed, 4)          # frontier / ray unknown area (1.0)
       + w_camera  * min(unseen_floor_m2_in_view, 4)      # CameraCoverage (0.8)
       + w_mission * inspection_priority                  # 0..3 (3.0)
       + w_grid    * unfinished_fraction_of_target_cell   # memory pull (1.5)
       + w_agree   * (n_sources - 1)                      # (0.3)
       - w_travel  * route_length_m                       # Nav2 path length if planned (0.4)
       - w_turn    * |turn_rad|                           # (0.3)
       - w_terrain * unknown_terrain_fraction_on_route    # terrain cloud / unknown cells (1.0)
       - w_revisit * (target cell visited and seen)       # (1.0)
       - w_fail    * temporary_failures_near_target       # (2.0)
```

   Weights in brackets are starting values. They get tuned on saved maps, not on the course.

### 2.4 One movement owner and reselection rules
- `RoamSession` stays the only node that sends, cancels or changes Nav2 goals, speed and gait. The grid wrappers are retired; `~/grid_roam*` stays untouched for reference.
- The decision runs when idle (every 0.5 s) and, while walking, re-evaluates every 2 s.
- A walking goal is **replaced** only when one of these holds:
  1. **Invalid goal:** the target became lethal or occupied, left the geofence, the map was reset, or the target cell is finished and the goal has no other source left.
  2. **Clearly better:** the new best ≥ 1.3 × current + 1.0, the current goal has run ≥ 6 s, it is > 1 m from its target, and there has been no switch in the last 10 s.
  3. **Inspection:** an inspection with priority ≥ 2 arrives. The same timing rules apply, with margin × 1.1.
- Replacement always means cancel, then wait for the terminal result (the existing `cancel_request` ack, 3 s pause on no ack), and only then send the new goal.
- **No reselection** while any of these is active: stairs/step crossing, curtain crossing, drift nudge, passage approach, advisory task, inspection wait, terrain hold, pending cancel, spin in progress. Safety stops and pauses discard the decision.
- The current goal is re-scored with the same terms (remaining distance), so the comparison is fair.

### 2.5 Recovery and completion
- **Stuck** means any of:
  - < 0.5 m net movement **and** < 0.5 m² new map or camera area in `--stuck-window` (default 60 s);
  - 3 consecutive failed goals;
  - no candidate scores above `--min-score`.
- **Recovery ladder**, each step bounded and logged:
  1. Clear costmaps.
  2. Full look-around in 45° steps (once per 1 m spot).
  3. **Scene-understanding request**: roam publishes `/exploration/scene_request` `{request_id, reason, pose, map_session}`. A responder takes **one** current RGB frame, asks the existing curtain/stairs/opening prompt, and returns `/exploration/scene_hints` `{request_id, openings:[{kind: curtain|gap|stairs|passage, image_region: left|centre|right, confidence}], image_quality}`. Hints become `scene_hint` candidates. A curtain goes to the existing curtain crossing; stairs go to the existing stairs routine, where AI mode is still used only on stairs. Limits: at most 1 request per 60 s and 6 per run, with a 20 s answer timeout.
  4. `grid_return` or `backtrack` to the nearest unfinished cell.
  5. Pause with a report.
- **Bounds:** the existing look and abandon limits, inspections (max 2 attempts, deferred retry 60 s), failures (temporary 3 / persistent 2).
- **Inspected** only on a usable-camera confirmation (unchanged rule). Cells get `seen` from camera coverage, never from being walked.
- **Completion:** roam never reports success from "nothing left to choose". The new outcome is `NO_REACHABLE_GOAL`, and the report lists unfinished cells, open branches, pending inspections and, when available, unresolved missions read from `pi_bridge`'s `status.json` (read-only).

### 2.6 RViz (same decision state, one MarkerArray)
`/exploration/status_markers`, throttled to 1 Hz (5 Hz max on change), DELETEALL first as today. New layers, each with short tags and its own namespace so it can be ticked off:
- `grid`: flat 1 m squares, 2 cm thick.
  - No fill when unmapped, grey when mapped, blue outline when visited, green fill by seen fraction.
  - **Orange "come back"** when unfinished.
  - Connections as thin lines: open, unknown dashed, blocked with an X.
- `decision`: the chosen goal (existing shapes) with the tag `GOAL <top reason>`, plus the next 4 alternatives as small spheres tagged with their score.
- `status`: one small banner above the robot, e.g. `ROAMING`, `HOLD`, `STEP CROSSING`, `STAIRS`, `RECOVERY: scene query`, `PAUSED: <why>`. That's the indicator for every kind of roam.
- Existing layers are unchanged: breadcrumbs, junctions/branches, failures, inspections, tags.

`operator/competition.rviz` (new) has all displays preconfigured: map, costmap, robot, scan, the status markers and `/live_mission/candidate_markers`.

### 2.7 One command: `operator/competition_start.sh`
```
operator/competition_start.sh [--physical | --dry | --sim] [--perception pi|local|none] [--no-rviz] [roam options...]
```
It runs:
1. preflight;
2. `stop_navigation.sh` if a different session is running;
3. navigation with the defaults (`LITE3_LIO_SOURCE=local`, max-step 0.35);
4. waits for `NAVIGATION READY`;
5. the perception you pick: Pi bridge (`pi_bridge.sh`) or `run_live.sh --rgbd --publish-mission-events`, in its own logged process;
6. the scene responder;
7. RViz with `competition.rviz`;
8. roam with `--strategy unified` and the `ARM` / `ROAM` prompts.

Ctrl+C or `competition_start.sh stop` stops roam first, then the helpers, then navigation. `operator/roam.sh` stays as is for single tests.

## 3. Files and interfaces

| File | Change |
|---|---|
| `src/lite3_movement_tests/lite3_movement_tests/course_grid.py` | **new**: `CourseGrid` (alignment, cells, connections, unfinished, `markers()`), with `corridor`/`crossing` adapted from `grid_roam_nav2` |
| `.../goal_selection.py` | **new**: `Candidate`, `merge()`, `evaluate(candidates, ctx, weights) -> Decision(chosen, ranked, breakdown)`, `Reselection` policy |
| `.../plan_check.py` | **new**: ROS plan-only reachability (`PlanOnly` adapted), with a sim stub |
| `.../scene_probe.py` | **new**: request/answer protocol (pure parser), plus the responder node if it runs on the NUC (§8.1) |
| `.../roam.py` | `ScanWalk.candidates()`; `Explorer.candidates()`; `RoamSession._decide()` replaces the fixed order in the goal-None branch; reselection in walk monitoring; stuck detector and recovery ladder; `NO_REACHABLE_GOAL`; config/CLI for `--strategy unified`, weights, grid, stuck window, scene limits; map snapshots in the run folder |
| `.../exploration_record.py` | Holds `grid` and `decision` (chosen + ranked + breakdown, bounded); `build_markers` adds the grid, alternatives and status banner; summary includes unfinished cells |
| `.../roam_ros.py`, `exploration_ros.py` | `scene_request` publisher, `scene_hints` subscriber, plan client, `markers_to_msg` unchanged |
| `.../roam_sim.py`, `sim_model.py` | Load a saved map snapshot; ComputePathToPose stub; scripted dropouts |
| `operator/competition_start.sh`, `operator/competition.rviz` | **new** |
| `operator/README.md`, `CHEATSHEET.md` | New section. **Another session is editing these now**, so I'll add my part last and only append |
| `~/live_mission/COMMANDS.md` | Navigation section, only with your OK (teammate's file) |
| Not touched | `~/grid_roam*`, `~/scene_understanding`, `~/stair_curtain`, vendor limits, dead-man lock, watchdog, 250 ms axis timeout, heartbeat, remote STOP |

## 4. Staged order (each stage ends green on tests; no stage moves the robot by itself)
0. **Saved maps.** Roam (and a small `operator/capture_map.sh`) writes `/map`, the costmap and pose snapshots (NDJSON, same format as the grid preview's `--replay`). Build 8 fixture maps: 4 synthetic, and real ones captured in the lab on your instruction.
1. **Course grid as memory only.** Pure module, markers and report. Live in dry runs. No movement change.
2. **Generators.** Refactor ScanWalk, Explorer, look, inspection and backtrack into candidate lists. Old behaviour stays identical (tests prove the chosen goal is unchanged).
3. **Evaluator in shadow mode.** It logs `decision_shadow` (what it would pick, with breakdown) next to what roam did, in sim and dry runs.
4. **Switch.** `--strategy unified` lets the evaluator choose; hysteresis and reselection added. `scan-walk` stays selectable as the fallback.
5. **Recovery.** Stuck detector, ladder, `NO_REACHABLE_GOAL`, scene request protocol and responder.
6. **RViz.** Grid, alternatives and status banner, plus `competition.rviz`.
7. **`competition_start.sh`** and docs.
8. **Physical tests**, only on your instruction, at normal speed.

## 5. Acceptance tests (saved maps + simulated movement, `test_competition_scenarios.py`)
| Scenario | Pass condition |
|---|---|
| Repeated dead ends | Each dead-end connection is marked blocked after its bounded attempts; no target is tried more than 2× (persistent) or 3× (temporary); the robot leaves via `grid_return` or `backtrack` |
| Reachable cell, blocked centre | The cell is reached via an off-centre point; it becomes visited and seen without the centre |
| Distant unfinished branch | After the local area is finished, `grid_return` is chosen through visited cells and the branch's cell turns seen |
| Mission candidate in a visited cell | An inspection in an already-visited cell is still chosen first (`mission` term) and needs a usable confirmation for INSPECTED |
| Sensor dropout and recovery | Terrain or map goes stale → hold or pause, no reselection; after recovery the same goal is resumed or re-chosen, no failure counted as persistent |
| Competing candidates | Two candidates within 20 % of each other over 60 s: ≤ 1 switch, no cancel storm (≤ 1 cancel per 10 s) |
| Map reset, stale requests | A session change clears grid, record and decisions; inspection requests with an old `map_session` are refused |
| No reachable goal | Outcome `NO_REACHABLE_GOAL` (never `COMPLETE`), report lists unfinished cells and pending inspections |
| Stuck → scene hint | A scripted hint "curtain centre" creates a candidate and hands off to the curtain crossing; requests limited to 1 per 60 s and 6 per run |
| Shapes and tags | Markers per layer bounded; all text ≤ 20 chars and ≤ 8 cm |

All existing suites stay green: 83 locomotion, 192 movement.

## 6. How you will start the tests (once implemented)
```bash
cd ~/lite3_ws && source operator/env.sh
python3 -m pytest -q -p no:cacheprovider src/lite3_movement_tests/test/test_competition_scenarios.py
```
```bash
operator/competition_start.sh --sim          # whole pipeline on the stand-in, RViz shows grid/goal/status
```
```bash
operator/competition_start.sh --dry          # real sensors, robot still: grid and decisions live in RViz
```
```bash
operator/competition_start.sh --physical     # type ARM, then ROAM; remote STOP in hand
```

## 7. Risks
- Plan-only Nav2 calls share the planner with the live goal. They're limited to 1 in flight and 3 per decision, with a 3 s timeout.
- The camera "seen" layer depends on SLAM wall cells. The low LiDAR band misses short objects, so "seen" can be optimistic near them.
- Scene requests cost API calls and add latency. Strict per-run limits apply, and roam never waits on them while walking.

## 8. Open dependencies (need a decision or a teammate)
1. **Where the scene responder runs.** On the NUC in `lite3_ws` (needs an API key on the NUC and one RGB frame from the robot camera), or in `pi_bridge`/the Pi (teammate code; the Pi key is missing and the Orbbec is unplugged as of 2026-11-24).
2. **Who publishes inspection requests and camera confirmations.** Either `pi_bridge`/`live_perception` (teammate change), or a lite3_ws adapter that turns `/mission/events` with a location into inspection requests. Without one of these, mission candidates are test-only.
3. **Which perception `competition_start` launches by default:** Pi bridge or local `run_live.sh --rgbd`.
4. **Real saved maps.** Capturing them needs navigation running in the lab (dry; the robot can be walked by hand).
