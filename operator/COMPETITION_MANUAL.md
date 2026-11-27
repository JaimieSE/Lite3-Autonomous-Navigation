# Competition run manual (group 8)

How to start, watch and end an official competition run: robot, Pi, RViz, scoring reporter and
monitoring dashboard. Practice runs use the same steps with one extra flag (section 7).

**The handheld remote STOP is the emergency stop.** Keep it in hand from `ARM` until the run is over.

| What | Where |
|---|---|
| Run everything robot-side | NUC `group8`, `~/lite3_ws`, in a terminal on the NUC's desktop (RViz needs a display) |
| Dashboard | your laptop, browser at http://127.0.0.1:3000 |
| Pi perception | `group8pi`, user service `group8pi-perception` |
| Scoring identity | `~/live_mission/group_id.txt` (format `group08_XXXX`) |

---

## 1. Checklist at a glance

1. Pi running and publishing (section 2).
2. Command logger running (section 2).
3. Dashboard live, **Record session** pressed (section 3).
4. Robot standing in the start cell, facing the course's +X direction, remote in Navigation (auto) mode (section 4).
5. `operator/competition_start.sh --physical --arena A` or `--arena B` (section 5).
6. Type `ARM`, then `ROAM`.
7. At the end: `operator/competition_start.sh stop`, dashboard **STOP**, collect logs (section 6).

---

## 2. NUC preparation

Open a terminal on the NUC:

```bash
cd ~/lite3_ws && source operator/env.sh
```

### 2.1 Pi perception

Without the Pi there is no stair detection, no scene checks and no mission decisions (M01–M05).

```bash
ssh group8pi 'systemctl --user is-active group8pi-perception'
```

It must print `active`.
- If it prints `inactive` or `failed`, start the service: `ssh group8pi 'systemctl --user restart group8pi-perception'`.
- If SSH fails with *No route to host* or a timeout, the Pi is off, under-powered (it is powered by USB-C from the
  NUC), or not on the RI_Network Wi-Fi. Fix that first.

Then confirm its messages reach the NUC:

```bash
ros2 topic info /group8pi/perception/detections
```

It must show `Publisher count: 1` (or more). `competition_start.sh` repeats this check and prints a WARNING if
the Pi is silent. Do not type `ROAM` past that warning in an official run.

### 2.2 Command logger

It records every command the robot receives, and who sent it. It is read-only.

```bash
operator/command_log.sh start
```

Check it with `operator/command_log.sh status`. It does not restart by itself after a reboot of the NUC.

### 2.3 Scoring identity

```bash
cat ~/live_mission/group_id.txt
```

It must be exactly `group08_` followed by 4 letters or digits. If not, the reporter silently stays in dry run
(prints payloads, posts nothing) and `competition_start.sh` prints a WARNING.

---

## 3. Monitoring dashboard (laptop)

The dashboard only watches. It can never move or stop the robot.

### 3.1 Bridge on the NUC (usually already running)

```bash
pgrep -af lite3_bridge
```

If that prints nothing, start it (one person only):

```bash
tmux attach -t monitor || tmux new -s monitor
```
```bash
~/Lite3_Monitoring_Dashboard/lite3-monitor/scripts/nuc/start-bridge.sh
```

Leave it running: **Ctrl+B**, release, then **D**.

### 3.2 Dashboard on each laptop

From `~/Lite3_Monitoring_Dashboard/lite3-monitor` on the laptop:

| Mac | Windows (PowerShell) |
|---|---|
| `./scripts/mac/start-all.sh` | `powershell -ExecutionPolicy Bypass -File scripts\windows\start-all.ps1` |

Type the NUC password when asked. Keep that window open.

### 3.3 Check it is live

| Where | Must show |
|---|---|
| Top bar, left | **Robot verified** · `group8` · domain 8 |
| Top bar, right | **LIVE** (green) and **REC** |
| Sensor chips | IMU, Joints, Odom, Odom2, Ultrasound all **alive** |
| Raspberry Pi panel | online |

Press **Record session** at the start of each run and **STOP** at its end, so each run is its own recording
(History tab, export JSON/CSV). Full dashboard manual: `~/Lite3_Monitoring_Dashboard/lite3-monitor/docs/STARTUP_MANUAL.md`.

If only a few people need it, others should close their tab: the dashboard shares the Wi-Fi with the robot.

---

## 4. Robot preparation

- Robot **standing**.
- In the **start cell**, facing the course's **+X direction**. The scoring reporter takes the arena origin from the
  robot's first pose, and roam aligns its 1 m course grid to it.
- Remote in **Navigation (auto) mode**, sticks untouched.
- Operator next to the course with the remote **STOP** in hand.
- No other group on our robot (domain 64), and no other test sending it commands. The command logger
  (`operator/command_log.sh show`) shows any foreign sender.

---

## 5. Start the run

### 5.1 Command

**Arena A**

```bash
operator/competition_start.sh --physical --arena A
```

**Arena B**

```bash
operator/competition_start.sh --physical --arena B
```

**Always give `--arena`.** Without it the reporter posts to Arena **B**. Use the organisers' arena label.

The command:
1. runs the read-only readiness check (output also in `preflight.txt`);
2. restarts navigation if it is running in the wrong mode;
3. checks the Pi is publishing;
4. starts the mission reporter in **live** mode: it decides M01–M05 and posts results to the arena's scoring
   endpoint as our `group_id` (it never moves the robot);
5. starts `nav_hold`, which lets a Pi scene check pause the robot briefly at a safe point;
6. opens RViz with `operator/competition.rviz`;
7. starts roam at regular speed (`normal`), unified decision, centre lanes, pauses that resume by themselves.

### 5.2 Arm and go

1. The terminal asks for **`ARM`**: type it. Navigation starts.
2. The terminal asks for **`ROAM`**: type it. Roam switches the robot to navigation (auto) mode and **the robot
   starts moving**.

### 5.3 RViz

RViz opens by itself on the NUC desktop. If the start terminal had no display it prints a NOTE. Then open it
yourself in another NUC terminal:

```bash
source ~/lite3_ws/operator/env.sh && rviz2 -d ~/lite3_ws/operator/competition.rviz
```

What it shows:
- map, global costmap, `/scan`, the Nav2 plan;
- roam's grid: orange = come back, green = seen, grey = mapped, blue outline = walked, red X = blocked connection;
- grey spheres with scores: the alternatives roam considered;
- the status line over the dog: `ROAMING`, `GOAL: …`, `LOOKING`, `STUCK: …`, `PAUSED: resume 12s`, `HOLD`,
  `STEP CROSSING`, `STAIRS`, `CURTAIN CROSSING`;
- mission candidate pins.

Use only this layout. Do not add a raw `/livox/points` display: it pulls point clouds over the robot link and
delays robot telemetry and commands.

---

## 6. During and after the run

| Action | Command |
|---|---|
| State of roam and its helpers | `operator/competition_start.sh status` |
| Pause (waits for you) | `operator/roam.sh pause` |
| Resume | `operator/roam.sh resume` |
| **End the run** | `operator/competition_start.sh stop` |
| **Emergency** | remote **STOP** (or switch auto mode off) |

`stop` ends roam, the reporter, `nav_hold` and navigation. If roam ends by itself, the reporter and `nav_hold`
stop with it; RViz and navigation stay up until `stop`.

### 6.1 Watch the reports live

In a second NUC terminal, as soon as the run has started:

```bash
~/live_mission/watch_reports.sh
```

It follows the current run (the manual fallback's run when that is on) and changes nothing. Ctrl+C stops it.

| Line | Meaning |
|---|---|
| `Run: … reporter: POST http://…/task_report_B` | the reporter is **live** on that arena (`DRY RUN` = practice, nothing posted) |
| `scan fallback-…` / `result fallback-…` | a room photo was taken / Claude's answer (room, findings, QR) |
| `SENT {…}` | a report was posted to the scorer (the seven keys) |
| `… HTTP 200 … "status":"accepted" … "score"` | the scorer's reply, with its score |
| `Mx pending: …` / `held` | not posted yet: still waiting for confirmed evidence (the reason is given) |
| `already accepted earlier in this run (not re-sent): [...]` | after a reporter restart: those tasks are done |

Only confirmed answers are posted. Each survivor is reported as soon as her room scan comes back: **M03** if
Claude saw a QR code in that room, **M05** if not. Every answer, posted or held, is also in
`…/candidates.jsonl` next to `mission_reports.jsonl`.

### 6.2 Manual scene fallback (when autonomy is stuck)

Drive by hand while the NUC still takes the room photos and reports them:

```bash
~/live_mission/scene_check.sh --fallback on --live
```

- Roam pauses (map, Nav2 and the A1 origin stay up). Switch the remote to manual and drive.
- Stop in a room, or in front of one facing into it: after 1 s still, one photo of that room is taken and
  reported. It fires once per visit; leave the cell and come back for another photo.
- The arena is taken from the running competition. A different `--arena` is refused unless you add
  `--arena-override`. Without `--live` it is a dry run.

Back to autonomy:

```bash
~/live_mission/scene_check.sh --fallback off
```

Then put the robot back in navigation (auto) mode on the remote and run `operator/roam.sh resume`. The
fallback never moves the robot and never resumes roam.

After the reporter's code changes mid-run: `~/live_mission/scene_check.sh --fallback on --live --restart-reporter`
(same run folder; answers already accepted are not re-sent).

Teleoperating from the start, without roam: section 7.

Then:
1. Dashboard: press **STOP**.
2. Note the arena and run number for the log name (Arena A / Arena B).
3. Check the logs below.

### Logs

| File | Contents |
|---|---|
| `log/operator/competition-<time>-physical/preflight.txt` | readiness check |
| `…/reporter.log` | mission reporter decisions |
| `…/mission_reports.jsonl` | every payload posted to the scoring endpoint |
| `…/candidates.jsonl` | every candidate answer, posted or held (with the reason) |
| `…/scene_fallback.log`, `…/reporter-fallback.log` | manual scene fallback: photos taken and its reporter |
| `…/nav_hold.log` | Pi scene holds |
| `log/operator/run-*/roam/*/report.md` | roam report (the newest folder) |
| `log/operator/run-*/roam/*/mission_events.json` | mission events seen during roam |
| `operator/command_log.sh show` | commands the robot received, and senders |

---

## 7. Practice run (no scoring)

Exactly like an official run, but the reporter only prints payloads and posts nothing:

```bash
operator/competition_start.sh --physical --arena A --dry-report
```

Teleoperated only, on a test course (no roam; you drive with the remote, photos and reports as in 6.2). Put
the dog in A1, square to the walls, facing +X; that pose sets A1 and the room grid:

```bash
operator/start_navigation.sh
```
```bash
~/live_mission/scene_check.sh --fallback on
```
```bash
~/live_mission/watch_reports.sh
```

Add `--live` to post (it scores against the real arena's answers). Finish with
`~/live_mission/scene_check.sh --fallback off` and `operator/competition_start.sh stop`.

Without the robot moving (real sensors, robot still, RViz shows what roam would do):

```bash
operator/competition_start.sh --arena A
```

Without the robot at all (software stand-in, no RViz):

```bash
operator/competition_start.sh --sim --sim-seconds 300
```

Practice runs follow the same rule as official ones: regular speed, never slow mode.

---

## 8. Troubleshooting

| Symptom | What to do |
|---|---|
| `WARNING: no Pi perception publisher` | Section 2.1. Restart the Pi service; check its power and Wi-Fi. |
| `WARNING: … group_id.txt is missing or not group08_XXXX` | Fix the file (section 2.3), `stop`, start again. |
| `Roam is already running` | `operator/competition_start.sh status`, then `stop` before starting again. |
| `REFUSED: navigation is running in dry mode` | `operator/competition_start.sh stop`, then start again. |
| Dashboard **CONNECTING** / **TELEMETRY LOST** | Bridge not running (section 3.1) or the laptop's Wi-Fi dropped. The robot is not affected. |
| Robot moves or changes posture with no command from us | Remote STOP. Then `operator/command_log.sh show --all` to see who sent what. |
| RViz shows nothing | Navigation not up yet (type `ARM`), or RViz opened before `env.sh` was sourced: reopen it (section 5.3). |
| Roam `SAFETY_STOP` | The session has ended. Read the reason in `operator/competition_start.sh status` before restarting. |

Options for unusual cases (pass after `--arena`): `--no-rviz`, `--no-reporter`, `--no-scene-hold`,
`--no-centre-lane`, `--missions FILE`, and any roam option such as `--max-minutes 30`.
`operator/competition_start.sh --help` lists them.
