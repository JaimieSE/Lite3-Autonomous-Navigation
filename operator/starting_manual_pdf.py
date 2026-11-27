#!/usr/bin/env python3
"""Build operator/COMPETITION_STARTING_MANUAL.pdf (final starting manual, every scenario, priority order).

Real text throughout (DejaVu fonts), commands in their own boxes with exact line breaks, so every command copies
out of the PDF as typed. Regenerate:  python3 operator/starting_manual_pdf.py   (needs: pip install --user reportlab)
"""
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (KeepTogether, PageBreak, Paragraph, Preformatted, SimpleDocTemplate, Spacer, Table,
                                TableStyle)

OUT = Path(__file__).resolve().parent / 'COMPETITION_STARTING_MANUAL.pdf'
F = '/usr/share/fonts/truetype/dejavu/'
pdfmetrics.registerFont(TTFont('Sans', F + 'DejaVuSans.ttf'))
pdfmetrics.registerFont(TTFont('Sans-B', F + 'DejaVuSans-Bold.ttf'))
pdfmetrics.registerFont(TTFont('Mono', F + 'DejaVuSansMono.ttf'))
pdfmetrics.registerFont(TTFont('Mono-B', F + 'DejaVuSansMono-Bold.ttf'))
pdfmetrics.registerFontFamily('Sans', normal='Sans', bold='Sans-B', italic='Sans', boldItalic='Sans-B')

INK = colors.HexColor('#1b1f24')
MUTED = colors.HexColor('#5b6470')
ACCENT = colors.HexColor('#0b5cad')
RED = colors.HexColor('#b3261e')
CODE_BG = colors.HexColor('#f3f5f8')
RULE = colors.HexColor('#d5dae1')

S = {
    'title': ParagraphStyle('title', fontName='Sans-B', fontSize=20, leading=24, textColor=INK, spaceAfter=4),
    'sub': ParagraphStyle('sub', fontName='Sans', fontSize=10, leading=14, textColor=MUTED, spaceAfter=10),
    'part': ParagraphStyle('part', fontName='Sans-B', fontSize=15, leading=19, textColor=ACCENT, spaceBefore=4,
                           spaceAfter=8),
    'h1': ParagraphStyle('h1', fontName='Sans-B', fontSize=12.5, leading=16, textColor=INK, spaceBefore=10,
                         spaceAfter=4, keepWithNext=1),
    'h2': ParagraphStyle('h2', fontName='Sans-B', fontSize=10.5, leading=14, textColor=INK, spaceBefore=7,
                         spaceAfter=2, keepWithNext=1),
    'p': ParagraphStyle('p', fontName='Sans', fontSize=9.2, leading=12.8, textColor=INK, spaceAfter=4,
                        alignment=TA_LEFT),
    'li': ParagraphStyle('li', fontName='Sans', fontSize=9.2, leading=12.8, textColor=INK, leftIndent=14,
                         bulletIndent=3, spaceAfter=1.5),
    'cell': ParagraphStyle('cell', fontName='Sans', fontSize=8.4, leading=11, textColor=INK),
    'cellh': ParagraphStyle('cellh', fontName='Sans-B', fontSize=8.4, leading=11, textColor=INK),
    'code': ParagraphStyle('code', fontName='Mono', fontSize=8.6, leading=11.4, textColor=INK),
    'warn': ParagraphStyle('warn', fontName='Sans-B', fontSize=9.6, leading=13.2, textColor=RED),
}

story = []


def T(text, style='p'):
    story.append(Paragraph(text, S[style]))


def H1(text):
    story.append(Paragraph(text, S['h1']))


def H2(text):
    story.append(Paragraph(text, S['h2']))


def L(*items, numbered=False):
    for i, it in enumerate(items, 1):
        story.append(Paragraph(it, S['li'], bulletText=f'{i}.' if numbered else '•'))
    story.append(Spacer(1, 3))


def C(*lines):
    """One command per box line, exact text, nothing else inside (copies cleanly)."""
    pre = Preformatted('\n'.join(lines), S['code'])
    t = Table([[pre]], colWidths=[174 * mm])
    t.setStyle(TableStyle([('BACKGROUND', (0, 0), (-1, -1), CODE_BG), ('BOX', (0, 0), (-1, -1), 0.5, RULE),
                           ('LEFTPADDING', (0, 0), (-1, -1), 7), ('RIGHTPADDING', (0, 0), (-1, -1), 7),
                           ('TOPPADDING', (0, 0), (-1, -1), 5), ('BOTTOMPADDING', (0, 0), (-1, -1), 5)]))
    lead = []
    while story and isinstance(story[-1], Paragraph) and len(lead) < 3:   # keep label (and heading) with its box
        lead.insert(0, story.pop())
    if not lead and story and isinstance(story[-1], KeepTogether):        # back-to-back boxes stay together
        lead = list(story.pop()._content)
    story.append(KeepTogether(lead + [t, Spacer(1, 5)]))


def code(s):
    return f'<font name="Mono" size="8.2">{s}</font>'


def TAB(rows, widths, header=True):
    data = [[Paragraph(c, S['cellh'] if (header and r == 0) else S['cell']) for c in row]
            for r, row in enumerate(rows)]
    t = Table(data, colWidths=[w * mm for w in widths], repeatRows=1 if header else 0)
    st = [('GRID', (0, 0), (-1, -1), 0.4, RULE), ('VALIGN', (0, 0), (-1, -1), 'TOP'),
          ('LEFTPADDING', (0, 0), (-1, -1), 4), ('RIGHTPADDING', (0, 0), (-1, -1), 4),
          ('TOPPADDING', (0, 0), (-1, -1), 2.5), ('BOTTOMPADDING', (0, 0), (-1, -1), 3)]
    if header:
        st.append(('BACKGROUND', (0, 0), (-1, 0), CODE_BG))
    t.setStyle(TableStyle(st))
    story.append(t)
    story.append(Spacer(1, 6))


def BOX(text, color=RED):
    t = Table([[Paragraph(text, S['warn'] if color == RED else S['p'])]], colWidths=[174 * mm])
    t.setStyle(TableStyle([('BOX', (0, 0), (-1, -1), 1.2, color), ('LEFTPADDING', (0, 0), (-1, -1), 8),
                           ('RIGHTPADDING', (0, 0), (-1, -1), 8), ('TOPPADDING', (0, 0), (-1, -1), 6),
                           ('BOTTOMPADDING', (0, 0), (-1, -1), 6)]))
    story.append(t)
    story.append(Spacer(1, 7))


# ---------------------------------------------------------------------------------------------------------------- cover
T('Lite3 Group 8 — Competition Starting Manual', 'title')
T('Final edition, 27 Nov 2026. Every scenario, most important first. Commands are plain text: select and copy any '
  'grey box straight into a terminal. Part A is what you need on competition day; Part B is reference.', 'sub')
BOX('THE HANDHELD REMOTE STOP IS THE EMERGENCY STOP. Keep it in hand from ARM until the run is over. '
    'roam.sh stop, Ctrl+C and competition_start.sh stop are commanded stops, not emergency stops.')

H1('Who runs what')
TAB([['What', 'Where'],
     ['Everything robot-side', f'NUC {code("group8")} (192.168.10.78), folder {code("~/lite3_ws")}, in a terminal on '
      'the NUC desktop (RViz needs a display)'],
     ['Pi perception', f'{code("ssh group8pi")}, user service {code("group8pi-perception")}; capture half '
      f'{code("~/group8-capture/run.sh")}'],
     ['Dashboard (watch only)', f'your laptop, browser at {code("http://127.0.0.1:3000")}'],
     ['Scoring identity', f'{code("~/live_mission/group_id.txt")} = {code("group08_XXXX")}'],
     ['Robot system computer', '192.168.10.64 (university): must be powered and running, nothing to do there']],
    [42, 132])

H1('Competition run in 14 steps')
L('Robot, NUC and Pi powered; Pi on RI_Network Wi-Fi (Pi is powered by USB-C from the NUC). <b>§1</b>',
  'Pi bridging checks pass: service <b>active</b>, publisher count ≥ 1 on the NUC, ~1 Hz. <b>§2</b>',
  'NUC terminal: source env, start the command logger, check group_id, scoring server answers. <b>§3</b>',
  'Dashboard LIVE, <b>Record session</b> pressed. <b>§4</b>',
  'Robot standing in the start cell (A1), square to the walls, facing course +X, remote in Navigation (auto) mode, '
  'sticks untouched. <b>§5</b>',
  f'{code("operator/competition_start.sh --physical --arena A")} (or {code("B")}). <b>§6</b>',
  'Read the start lines: Pi publishing, reporter LIVE, Pi capture started. No WARNING. <b>§6.2</b>',
  f'Type {code("ARM")} (navigation starts), then {code("ROAM")} (robot moves). <b>§6.3</b>',
  f'Second terminal: {code("~/live_mission/watch_reports.sh")}; third: server POST log. <b>§7</b>',
  'Watch RViz and the robot. Remote STOP in hand.',
  'Autonomy stuck? Manual takeover for scene understanding. <b>§8</b>',
  f'End: {code("operator/competition_start.sh stop")}. <b>§10</b>',
  'Dashboard <b>STOP</b>. Note "Arena A" / "Arena B" and run number for the logs.',
  'Copy the run folder and roam report. <b>§10.2</b>', numbered=True)

story.append(PageBreak())

# ------------------------------------------------------------------------------------------------------------- PART A
T('Part A — Competition day', 'part')

H1('1. Power and network')
L('Robot on and booted; system computer 192.168.10.64 up (the university domain bridge must be running).',
  'NUC up; you are logged in on its desktop (or a session with a display for RViz).',
  'Pi powered by USB-C from the NUC, joined to the <b>RI_Network</b> Wi-Fi. The Pi sits on 192.168.20.x, the NUC on '
  '192.168.10.78: different subnets, so the Pi reaches the NUC only through the unicast peer in its '
  f'{code("nuc-dds.xml")}.',
  'Nobody else on our robot (no other group, no other test sending it commands).')

H1('2. Raspberry Pi bridging checks (do all of them, every run)')
T('Without the Pi there is no stair/curtain/roller detection, no room scans (Claude scene understanding), no QR '
  'reading and therefore no M01–M05 reports. Run these on the NUC.')
H2('2.1 Pi reachable and perception service active')
C("cd ~/lite3_ws && source operator/env.sh")
C("ssh group8pi 'systemctl --user is-active group8pi-perception'")
T(f'Must print {code("active")}. If {code("inactive")} or {code("failed")}:')
C("ssh group8pi 'systemctl --user restart group8pi-perception'")
T('Read why it failed (last 40 lines of the service log):')
C("ssh group8pi 'journalctl --user -u group8pi-perception -n 40 --no-pager'")
T(f'If SSH says <i>No route to host</i> or times out: the Pi is off, under-powered or not on RI_Network. Fix that '
  'first; nothing else in this section can pass.')

H2('2.2 Pi messages arrive on the NUC (the actual bridge)')
C("ros2 topic info /group8pi/perception/detections")
T(f'Must show {code("Publisher count: 1")} or more. Then check the rate:')
C("ros2 topic hz /group8pi/perception/detections")
T('Expect about <b>1 Hz</b> (obstacle YOLO for stairs/curtain/roller, 1.1–1.2 Hz). Ctrl+C to leave. Room-scan '
  'results come back on a second topic:')
C("ros2 topic info /group8pi/perception/room_scans")
TAB([['Result', 'Meaning / fix'],
     ['Service active, publisher count 0', 'DDS bridge broken: the Pi cannot reach the NUC. Check both on RI_Network, '
      'restart the Pi service (2.1), wait 10 s, check again. Persisting: the unicast peer in the Pi '
      f'{code("nuc-dds.xml")} must still point at 192.168.10.78.'],
     ['Publisher count ≥ 1 but no messages in hz', 'Pi camera or detector stalled: restart the service, read the '
      'journal (2.1).'],
     ['Well under 1 Hz', 'Pi overloaded or Wi-Fi weak. Stairs are seen less often. Restart the service; keep the '
      'person/tool/hazard detectors <b>off</b> (they are off by design since 27 Nov).']],
    [55, 119])
H2('2.3 Pi capture half (photo → Claude → answer)')
T('competition_start.sh starts it by itself and prints <i>pi capture started</i>. To check by hand:')
C("ssh group8pi '~/group8-capture/run.sh status'")
T('Expected behaviour on the Pi: obstacle YOLO, local QR decoding and Claude room scans requested by roam. Person, '
  'hand-tool and hazard YOLO are intentionally <b>off</b>, so expect no scene holds while walking; survivors and '
  'objects are found by the room scans.')
BOX('Official run: do not type ROAM past "WARNING: no Pi perception publisher". Fix the Pi first.')

H1('3. NUC preparation')
C("cd ~/lite3_ws && source operator/env.sh")
H2('3.1 Command logger (records every command the robot receives, read-only)')
C("operator/command_log.sh start")
C("operator/command_log.sh status")
T('It does not restart by itself after a NUC reboot.')
H2('3.2 Scoring identity')
C("cat ~/live_mission/group_id.txt")
T(f'Exactly {code("group08_")} + 4 letters/digits. Otherwise the reporter silently stays in DRY RUN (posts '
  'nothing) and the start prints a WARNING.')
H2('3.3 LiDAR fragment fix (installed permanently; just confirm)')
C("sysctl net.ipv4.ipfrag_high_thresh")
T(f'Must print {code("67108864")}. If it shows 4194304, apply the fix in §17.')
H2('3.4 Scoring server reachable (before ARM)')
T('The reporter posts to 192.168.10.141, port 8001. This sends nothing to the scorer; it only asks whether the '
  'server answers. Use your arena letter:')
C("curl -s -o /dev/null -w '%{http_code}\\n' -m 5 http://192.168.10.141:8001/task_report_A")
C("curl -s -o /dev/null -w '%{http_code}\\n' -m 5 http://192.168.10.141:8001/task_report_B")
TAB([['Prints', 'Meaning'],
     ['any 3-digit code (200, 404, 405, 422 …)', 'server is up and reachable: go on'],
     [code('000'), 'not reachable: check the NUC network (RI_Network / lab LAN) and that the organisers\' '
      'server is running. Do not start an official run until it answers.']], [62, 112])

H1('4. Monitoring dashboard (laptop)')
T('It only watches; it can never move or stop the robot.')
H2('4.1 Bridge on the NUC (usually already running)')
C("pgrep -af lite3_bridge")
T('If that prints nothing (one person only):')
C("tmux attach -t monitor || tmux new -s monitor")
C("~/Lite3_Monitoring_Dashboard/lite3-monitor/scripts/nuc/start-bridge.sh")
T('Detach and leave it running: <b>Ctrl+B</b>, release, then <b>D</b>.')
H2('4.2 Dashboard on each laptop (from ~/Lite3_Monitoring_Dashboard/lite3-monitor)')
T('Mac:')
C("./scripts/mac/start-all.sh")
T('Windows (PowerShell):')
C("powershell -ExecutionPolicy Bypass -File scripts\\windows\\start-all.ps1")
T('Type the NUC password when asked; keep that window open.')
TAB([['Where', 'Must show'],
     ['Top bar, left', '<b>Robot verified</b> · group8 · domain 8'],
     ['Top bar, right', '<b>LIVE</b> (green) and <b>REC</b>'],
     ['Sensor chips', 'IMU, Joints, Odom, Odom2, Ultrasound all <b>alive</b>'],
     ['Raspberry Pi panel', 'online']], [45, 129])
T('Press <b>Record session</b> at the start of each run and <b>STOP</b> at its end. Only the people who need it keep '
  'a tab open: the dashboard shares the Wi-Fi with the robot.')

H1('5. Robot placement')
L('Robot <b>standing</b>.',
  'In the <b>start cell (A1)</b>, square to the walls, facing the course <b>+X</b> direction. The reporter takes the '
  'arena origin from the robot\'s first pose; roam aligns its 1 m course grid to it. A wrong start pose means wrong '
  'room cells in every report.',
  'Remote in <b>Navigation (auto)</b> mode, sticks untouched (stick input stops roam).',
  'Operator beside the course, remote STOP in hand.')

H1('6. Start the run')
H2('6.1 Command (always give --arena; without it the reporter posts to Arena B)')
T('Arena A:')
C("operator/competition_start.sh --physical --arena A")
T('Arena B:')
C("operator/competition_start.sh --physical --arena B")
H2('6.2 Lines to see before typing anything')
TAB([['Line', 'Meaning'],
     [code('== Pi perception: publishing'), 'Pi bridge OK (§2).'],
     [code('== Pi mission reporter (LIVE: POST to task_report_A as group08_...)'), 'Scoring live on the right arena. '
      '"DRY RUN" here on an official run = group_id problem (§3.2).'],
     [code('pi capture started ...'), 'Room-scan photos will get a Claude reading.'],
     [code('== RViz (operator/competition.rviz)'), 'RViz opening on the NUC desktop.'],
     [code('WARNING: ...'), 'Stop and fix it (§11) before ROAM.']], [92, 82])
T('The command: runs the readiness check, restarts navigation if it is in the wrong mode, checks the Pi, starts the '
  'mission reporter (decides M01–M05, never moves the robot), nav_hold, RViz stair labels, the camera capture '
  '(NUC relay + Pi), RViz, then roam at regular speed with the grid decision, centre lanes and Pi-only stair '
  'hand-over.')
H2('6.3 Arm and go')
L(f'Prompt asks for {code("ARM")}: type it. Navigation starts (30–60 s).',
  f'Prompt asks for {code("ROAM")}: type it. Roam switches the robot to navigation (auto) mode and '
  '<b>the robot starts moving</b>.', numbered=True)
H2('6.4 RViz')
T('Opens by itself. If the start printed a NOTE (no display), open it in another NUC desktop terminal:')
C("source ~/lite3_ws/operator/env.sh && rviz2 -d ~/lite3_ws/operator/competition.rviz")
T('Grid colours: orange = come back, green = seen / corridor path, grey = mapped, blue outline = walked, red X = '
  'blocked connection. Status line over the dog: ROAMING, GOAL, LOOKING, STUCK, PAUSED: resume 12s, HOLD, STEP '
  'CROSSING, STAIRS, CURTAIN CROSSING. Use only this layout: never add a raw /livox/points display (it delays robot '
  'telemetry and commands).')

H1('7. Watch the scoring live')
T('Second NUC terminal, as soon as the run has started:')
C("~/live_mission/watch_reports.sh")
TAB([['Line', 'Meaning'],
     [code('Run: … reporter: POST http://…/task_report_B'), 'reporter is <b>live</b> on that arena (DRY RUN = '
      'nothing posted)'],
     [code('scan fallback-…') + ' / ' + code('result fallback-…'), 'a room photo taken / Claude\'s answer (room, '
      'findings, QR)'],
     [code('SENT {…}'), 'a report was posted to the scorer'],
     [code('ACCEPTED … "score"'), 'scorer accepted it, with its score'],
     [code('Mx pending: …') + ' / ' + code('held'), 'not posted yet: waiting for confirmed evidence (reason given)'],
     [code('already accepted earlier in this run'), 'after a reporter restart: those tasks are done, not re-sent']],
    [80, 94])
T('Only confirmed answers are posted. Each survivor is reported when her room scan comes back: <b>M03</b> if Claude '
  'saw a QR code in that room, <b>M05</b> if not. Ctrl+C stops watching; it changes nothing.')
H2('7.1 Is the server being reached? (every POST attempt, failures included)')
T(f'{code("watch_reports.sh")} hides failed attempts, so a server that is down looks like a quiet run. Third NUC '
  'terminal:')
C('run="$(ls -td ~/lite3_ws/log/operator/competition-* | head -1)"',
  'tail -F "$run"/reporter*.log | grep --line-buffered -E "POST|HTTP [0-9]"')
TAB([['Line', 'Meaning'],
     [code('M0x HTTP 200: {"status":"accepted" …'), 'server reached, report accepted (with score)'],
     [code('M0x HTTP 4xx/5xx: …'), 'server reached but it refused the report; the reason is in the text'],
     [code('M0x POST failed (…); retrying in 2s'), 'server <b>not reached</b>. The reporter keeps retrying the '
      'same report, nothing is lost. Check the network to 192.168.10.141 (§3.4).'],
     [code('POST http://192.168.10.141:8001/…'), 'reporter start line: live, on that arena'],
     ['nothing at all for minutes', 'no confirmed answers yet (normal early in a run), or the reporter is not '
      f'running: {code("operator/competition_start.sh status")}']], [72, 102])

story.append(PageBreak())
H1('8. Manual takeover for scene understanding')
BOX('Use this when autonomy is stuck, looping, or will not reach rooms. You drive with the remote; the NUC still '
    'takes the room photos, gets Claude\'s scene answer and posts the reports. Map, Nav2 and the A1 origin stay up.',
    ACCENT)
H2('8.1 Take over')
L('Turn the fallback on (it pauses roam itself):', numbered=False)
C("~/live_mission/scene_check.sh --fallback on --live")
L('Switch the remote to <b>manual</b> and drive.',
  'Stop <b>inside a room</b>, or in the cell <b>in front of a room facing straight into it</b> (within a few degrees of '
  'a grid axis). Hold still <b>1 s</b>: one centre photo of that room is taken and reported.',
  'One photo per visit: leave the cell and come back for another. Blurry/failed captures are retried automatically.',
  f'Watch {code("watch_reports.sh")} (§7) for scan → result → SENT → ACCEPTED.', numbered=True)
T(f'Without {code("--live")} it is a dry run (nothing posted). The arena comes from the running competition; a '
  f'different {code("--arena")} is refused unless you add {code("--arena-override")}. No scan (reason in '
  f'{code("scene_fallback.log")}) when: pose stale, localization lost, no grid yet, or an ambiguous doorway.')
H2('8.2 Hand back to autonomy')
C("~/live_mission/scene_check.sh --fallback off")
T('Put the robot back in Navigation (auto) mode on the remote (or with the command below), then resume:')
C("operator/lite3_auto.sh")
C("operator/roam.sh resume")
T('The fallback never moves the robot and never resumes roam by itself.')
H2('8.3 Roam ended during takeover (e.g. remote stick input)')
T('The mission reporter keeps running after roam ends ("posting continues in manual"). Keep driving with the '
  'fallback on until the run is over, then §10. Do not run competition_start.sh again mid-run: it starts a new '
  'reporter and run folder.')
H2('8.4 Reporter code changed mid-run')
C("~/live_mission/scene_check.sh --fallback on --live --restart-reporter")
T('Same run folder; answers already accepted are not re-sent.')
H2('8.5 Teleoperated from the start (no roam at all)')
T('Dog in A1, square to the walls, facing +X: that pose sets A1 and the room grid. Three terminals:')
C("operator/start_navigation.sh")
C("~/live_mission/scene_check.sh --fallback on --arena A --live")
C("~/live_mission/watch_reports.sh")
T(f'Drive and stop at rooms as in 8.1 (rooms you drive into still count). Leave out {code("--live")} for practice. '
  'Finish with:')
C("~/live_mission/scene_check.sh --fallback off")
C("operator/competition_start.sh stop")
H2('8.6 One manual room scan for a cell you name (no roam running)')
T('Robot standing at/facing the room. Cell i,j on the arena grid: start cell (0,0), +X = start heading, +Y = left. '
  'Starts its own reporter, so use it only when no competition run is up.')
C("~/live_mission/scene_check.sh --cell 1,0 --report live --arena A")
T('Three views (turn the camera between views, Enter for the next):')
C("~/live_mission/scene_check.sh --cell 1,0 --views centre,left,right --report dry")
T('Scan by itself whenever a new QR code shows up (Ctrl+C to end):')
C("~/live_mission/scene_check.sh --cell 1,0 --auto --report live --arena A")

H1('9. Controls during the run')
TAB([['Action', 'Command'],
     ['State of roam and helpers', code('operator/competition_start.sh status')],
     ['Pause (waits for you)', code('operator/roam.sh pause')],
     ['Resume', code('operator/roam.sh resume')],
     ['Cross stairs by hand (from PAUSED, squarely facing them)', code('operator/roam.sh stairs up') + '  or  ' +
      code('operator/roam.sh stairs down')],
     ['Robot to navigation (auto) mode', code('operator/lite3_auto.sh')],
     ['<b>End the run</b>', code('operator/competition_start.sh stop')],
     ['<b>Emergency</b>', '<b>Remote STOP</b> (or switch auto mode off)']], [70, 104])
T('Speeds and turn rates are locked for competition (regular speed, 0.5 rad/s in-place turns). Do not change them '
  'during a run. Roller (20 cm) is walked over, never avoided. Stuck on a tilt: roam resumes straight away; do not '
  'pause to level it.')

H1('10. End of run')
H2('10.1 Stop')
C("operator/competition_start.sh stop")
T('Ends roam, reporter, nav_hold, capture (NUC and Pi) and navigation. Then dashboard <b>STOP</b>, note '
  '"Arena A" or "Arena B" (A is the harder course) and the run number.')
H2('10.2 Logs')
TAB([['File', 'Contents'],
     [code('log/operator/competition-<time>-physical/preflight.txt'), 'readiness check'],
     [code('…/reporter.log'), 'mission reporter decisions'],
     [code('…/mission_reports.jsonl'), 'every payload posted to the scorer'],
     [code('…/candidates.jsonl'), 'every candidate answer, posted or held (with reason)'],
     [code('…/scene_fallback.log') + ', ' + code('…/reporter-fallback.log'), 'manual takeover photos and reporter'],
     [code('…/nav_hold.log'), 'Pi scene holds'],
     [code('log/operator/run-*/roam/*/report.md'), 'roam report (newest folder); raw terminal in terminal_full.log'],
     [code('log/operator/run-*/roam/*/mission_events.json'), 'mission events seen during roam'],
     [code('operator/command_log.sh show'), 'commands the robot received, and senders']], [100, 74])
T('Newest roam report:')
C('cat "$(ls -td ~/lite3_ws/log/operator/run-*/roam/*/ | head -1)report.md"')

H1('11. Troubleshooting')
TAB([['Symptom', 'What to do'],
     [code('WARNING: no Pi perception publisher'), '§2: restart the Pi service; check power and Wi-Fi.'],
     [code('WARNING: Pi capture not started'), 'Pi reachable (§2.1)? Then check the capture half (§2.3). Without it, '
      'rooms get no Claude reading.'],
     [code('POST failed … retrying') + ' in reporter.log', 'Scoring server not reached. Run the §3.4 check; fix the '
      'network. The reporter retries by itself, no restart needed.'],
     [code('WARNING: … group_id.txt is missing'),'Fix the file (§3.2), stop, start again.'],
     [code('Roam is already running'), code('operator/competition_start.sh status') + ', then ' + code('stop') +
      ' before starting again.'],
     [code('REFUSED: navigation is running in dry mode'), code('operator/competition_start.sh stop') +
      ', then start again.'],
     ['Readiness NOT READY / LiDAR Hz low', 'Read the -> lines. Check §3.3 (ipfrag). Robot telemetry 0 Hz = robot or '
      '192.168.10.64 bridge down.'],
     ['Dashboard CONNECTING / TELEMETRY LOST', 'Bridge not running (§4.1) or laptop Wi-Fi dropped. Robot unaffected.'],
     ['Robot moves with no command from us', f'Remote STOP. Then {code("operator/command_log.sh show --all")}.'],
     ['RViz shows nothing', 'Navigation not up yet (type ARM), or reopen RViz with env sourced (§6.4).'],
     ['Roam SAFETY_STOP', 'Session ended. Read the reason in competition_start.sh status; take over manually (§8) or '
      'stop and restart only before the run has scored.'],
     ['Room photo never taken in takeover', 'Hold still a full second, face straight into the room, check '
      f'{code("scene_fallback.log")} for the reason.'],
     ['Stack half-dead after a crash', code('operator/stop_navigation.sh') + ' (safe any time), then start again.']],
    [62, 112])

story.append(PageBreak())
# ------------------------------------------------------------------------------------------------------------- PART B
T('Part B — Reference (less often needed)', 'part')

H1('12. Practice runs')
T('Exactly like an official run, reporter prints payloads only:')
C("operator/competition_start.sh --physical --arena A --dry-report")
T('Real sensors, robot still, RViz shows what roam would do:')
C("operator/competition_start.sh --arena A")
T('No robot at all (software stand-in, no RViz):')
C("operator/competition_start.sh --sim --sim-seconds 300")
T('Practice runs follow the same rule as official ones: regular speed, never slow mode.')

H1('13. Arena-holds test variant')
T('The competition run plus --arena-holds (face the next cell before each grid step, post-stairs re-centring, '
  'checked look-around headings). For comparison runs only; competition_start.sh does not enable it.')
C("operator/arena_holds.sh --physical --arena A")
C("operator/arena_holds.sh status")
C("operator/arena_holds.sh stop")

H1('14. Start options')
T('Pass after --arena. List them all:')
C("operator/competition_start.sh --help")
TAB([['Option', 'Effect'],
     [code('--no-rviz'), 'do not open RViz'],
     [code('--no-reporter'), 'no mission reporter (nothing decided or posted)'],
     [code('--no-scene-hold'), 'Pi scene checks never pause the dog'],
     [code('--no-capture'), 'no camera capture: rooms get no Claude reading (roam waits 12 s, moves on)'],
     [code('--no-centre-lane'), 'walk goals directly instead of cell centre to cell centre'],
     [code('--missions FILE'), 'local missions JSON for the reporter'],
     [code('--max-minutes 30'), 'any roam option, e.g. a time limit'],
     [code('--terminal full'), 'detailed terminal output instead of the plain lines'],
     [code('--decision unified|sequence'), 'older decision modes (default: grid)'],
     [code('--stairs-on-sight / --defer-stairs'), 'confirmed stair paths instead of Pi-only hand-over']], [62, 112])

H1('15. Navigation only, and status')
C("operator/preflight.sh")
C("operator/start_navigation.sh")
C("operator/status.sh --watch 10")
C("operator/stop_navigation.sh")
T('start_navigation.sh is dry (non-actuating) unless --physical. stop_navigation.sh is safe at any time and finds '
  'leftovers from crashed terminals.')
H2('After any code change')
C("operator/build.sh")

H1('16. Command logger details')
C("operator/command_log.sh show -n 50")
C("operator/command_log.sh show --all")
C("operator/command_log.sh stop")
T(f'Records: {code("log/operator/command-log/commands-YYYYMMDD.jsonl")}.')

H1('17. LiDAR IP-fragment fix')
T('Installed on this NUC from 27 Nov; survives reboots. Re-install if §3.3 shows the default:')
C("sudo install -m 644 operator/90-lidar-ipfrag.conf /etc/sysctl.d/ && sudo sysctl --system")
T('Verify:')
C("sysctl net.ipv4.ipfrag_high_thresh net.ipv4.ipfrag_low_thresh net.ipv4.ipfrag_time")
C("grep FRAG /proc/net/sockstat")
T('Revert to kernel defaults:')
C("sudo sysctl -w net.ipv4.ipfrag_low_thresh=3145728 net.ipv4.ipfrag_time=30",
  "sudo sysctl -w net.ipv4.ipfrag_high_thresh=4194304")

H1('18. Speed and gait (do not change without team agreement)')
C("operator/roam.sh modes")
C("operator/roam.sh plan")
T(f'Exist but are locked for competition: {code("operator/roam.sh speed slow|normal|fast")}, '
  f'{code("operator/roam.sh gait NAME")} (while paused). Default gait rug_hstep.')

H1('19. System-host FAST-LIO (alternative mapper)')
T('Default is local FAST-LIO on the NUC. To use the mapper on 192.168.10.64: stop navigation, then in every operator '
  'shell for that session:')
C("export LITE3_LIO_SOURCE=system")
T('After a system-host reboot, start the mapper there:')
C('tmux new-session -d -s group8_lio "cd ~/group8_lio && ./start.sh > run.log 2>&1"')

H1('20. Plain roam (no competition helpers)')
C("operator/roam.sh start --sim")
C("operator/roam.sh start --physical")
C("operator/roam.sh status")
C("operator/roam.sh stop")


def on_page(canvas, doc):
    canvas.saveState()
    canvas.setFont('Sans', 7.5)
    canvas.setFillColor(MUTED)
    canvas.drawString(18 * mm, 10 * mm, 'Lite3 Group 8 — Competition Starting Manual — 27 Nov 2026')
    canvas.drawRightString(192 * mm, 10 * mm, f'Page {doc.page}   ·   Emergency: remote STOP')
    canvas.restoreState()


doc = SimpleDocTemplate(str(OUT), pagesize=A4, leftMargin=18 * mm, rightMargin=18 * mm, topMargin=16 * mm,
                        bottomMargin=18 * mm, title='Lite3 Group 8 — Competition Starting Manual',
                        author='Group 8', subject='Competition start, manual takeover, Pi checks')
doc.build(story, onFirstPage=on_page, onLaterPages=on_page)
print(OUT)
