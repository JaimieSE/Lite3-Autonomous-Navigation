"""One goal-selection system (operator 2026-11-24), full roam loop on the simulator: one evaluator decides, the course
grid is memory, stuck ladder (look around -> scene understanding on the Pi -> unfinished cell), pauses resume by
themselves, mission events in a JSON file."""
import json
import math

from lite3_movement_tests import goal_selection as GS
from lite3_movement_tests import roam as R
from lite3_movement_tests.course_grid import CourseGrid
from lite3_movement_tests.roam_sim import SimRoamRobot
from test_m10 import roam


def cfg(**kw):
    kw.setdefault('auto_resume_s', 15.0)
    return R.RoamConfig(strategy='scan_walk', stop_at_exit=False, start_mode='normal', decision='unified', **kw)


def names(summary):
    return [e['event'] for e in summary['events']]


def first(summary, name):
    return next(e for e in summary['events'] if e['event'] == name)


def keep_session(monkeypatch):
    box = {}
    original = R.RoamSession._finish

    def finish(self, summary, outcome):
        box['s'] = self
        return original(self, summary, outcome)
    monkeypatch.setattr(R.RoamSession, '_finish', finish)
    return box


def no_goal_until(monkeypatch, until_s):
    """Nothing worth walking to (the evaluator finds no candidate) for the first until_s seconds of roaming."""
    original = R.RoamSession._evaluate

    def evaluate(self, now, pose, note=True):
        if now - self.t_roam < until_s:
            return GS.Decision(None, 0.0, {}, [])
        return original(self, now, pose, note)
    monkeypatch.setattr(R.RoamSession, '_evaluate', evaluate)


def pi_link(monkeypatch, answer=None):
    """The Pi scene link: requests are recorded; `answer` (a Pi message dict) arrives right after each request."""
    sent = []

    def publish(self, text):
        sent.append(json.loads(text))
        if answer is not None:
            self.pi_scene_messages.append((self.now() + 0.5, json.dumps(answer)))
    monkeypatch.setattr(SimRoamRobot, 'pi_scene_messages', property(
        lambda self: self.__dict__.setdefault('_pi', [])), raising=False)
    monkeypatch.setattr(SimRoamRobot, 'publish_scene_request', publish, raising=False)
    return sent


def test_one_evaluator_explores_with_logged_reasons_and_grid_memory(tmp_path, monkeypatch):
    box = keep_session(monkeypatch)
    outcome, summary, _, _ = roam(tmp_path, stop_after=200, cfg=cfg())
    st = summary['stats']
    # at most one terrain retry: return points may sit 0.35 m from a wall, which the simulator's straight-line "Nav2"
    # cannot always reach (the real Nav2 plans round it)
    assert st['decisions'] >= 3 and st['goals_failed'] <= 1 and st['distance_m'] > 8
    assert all(e['category'] == 'terrain' for e in summary['events'] if e['event'] == 'destination_failed')
    d = first(summary, 'decision')
    assert d['chosen']['terms'] and 'score' in d['chosen'] and isinstance(d['alternatives'], list)
    grid = summary['course_grid']
    assert grid['cells_visited'] >= 3 and grid['cells_mapped'] > grid['cells_visited']
    s = box['s']
    seen = [c for c, i in s.course.cells.items() if i['seen'] >= 0.7 and not i['visited']]
    assert seen                                                         # looked at without walking there
    assert 'paused' not in names(summary)


def test_reselection_does_not_thrash(tmp_path):
    _, summary, _, _ = roam(tmp_path, stop_after=300, cfg=cfg())
    times = [e['t_s'] for e in summary['events'] if e['event'] == 'reselect']
    assert len(times) <= max(1, summary['stats']['walk_goals'] // 2)
    assert all(b - a >= GS.Reselection().gap_s for a, b in zip(times, times[1:]))


def test_stuck_asks_the_pi_after_looking_around_and_crosses_the_curtain_it_reports(tmp_path, monkeypatch):
    sent = pi_link(monkeypatch, answer={'frame': 9, 'stair_curtain': {'obstacle': 'curtain', 'reason': 'confirmed',
                                                                       'heading_deg': 0.0, 'distance_m': 0.8}})
    no_goal_until(monkeypatch, 60.0)
    _, summary, _, _ = roam(tmp_path, stop_after=90, cfg=cfg())
    n = names(summary)
    assert 'spin' in n and n.index('spin') < n.index('scene_request')      # looked around first
    assert sent and sent[0]['request_id'] == 'scene-1' and 'pose' in sent[0]
    hint = first(summary, 'scene_hint')
    assert hint['hints'][0]['kind'] == 'curtain'
    crossing = first(summary, 'curtain_crossing')
    assert crossing['source'] == 'scene' and n.index('scene_hint') < n.index('curtain_crossing')


def test_without_a_scene_answer_it_heads_back_to_an_unfinished_cell(tmp_path, monkeypatch):
    sent = pi_link(monkeypatch, answer=None)
    original = R.RoamSession._evaluate
    state = {'stuck_at': None}

    def evaluate(self, now, pose, note=True):      # explore normally, then find nothing worth walking to
        if now - self.t_roam > 40 and state['stuck_at'] is None:
            state['stuck_at'] = now
        if state['stuck_at'] is not None and now - state['stuck_at'] < 150:
            return GS.Decision(None, 0.0, {}, [])
        return original(self, now, pose, note)
    monkeypatch.setattr(R.RoamSession, '_evaluate', evaluate)
    _, summary, _, _ = roam(tmp_path, stop_after=260, cfg=cfg(scene_timeout_s=5.0))
    n = names(summary)
    assert sent and 'scene_no_hint' in n
    back = [e for e in summary['events'] if e['event'] == 'stuck_step' and e['step'] == 'grid_return' and 'cell' in e]
    assert back and first(summary, 'scene_no_hint')['t_s'] <= back[0]['t_s']
    walk = [e for e in summary['events'] if e['event'] == 'walk' and e['ends_at'] == 'grid_return']
    assert walk


def test_scene_requests_are_bounded(tmp_path, monkeypatch):
    sent = pi_link(monkeypatch, answer=None)
    no_goal_until(monkeypatch, 1e9)
    _, summary, _, _ = roam(tmp_path, stop_after=200, cfg=cfg(scene_timeout_s=3.0, scene_interval_s=60.0,
                                                               scene_max=2))
    times = [e['t_s'] for e in summary['events'] if e['event'] == 'scene_request']
    assert 1 <= len(sent) == len(times) <= 2
    assert all(b - a >= 60.0 for a, b in zip(times, times[1:]))


def test_repeated_dead_ends_go_down_the_ladder_and_pauses_resume_by_themselves(tmp_path, monkeypatch):
    """Every walk is cut short (persistent obstacle): no operator pause; the ladder runs, the pause it ends with
    resumes by itself with a growing wait, and the session never claims success."""
    original = R.RoamSession._end_walk

    def cut_short(self, now, pose, how):
        if how == 'reached':
            self.walk_start = pose
            how = 'obstacle ahead (walking standoff)'
        return original(self, now, pose, how)
    monkeypatch.setattr(R.RoamSession, '_end_walk', cut_short)
    outcome, summary, _, _ = roam(tmp_path, stop_after=400, cfg=cfg(auto_resume_s=10.0))
    n = names(summary)
    assert 'stuck' in n and 'auto_resumed' in n
    pauses = [e for e in summary['events'] if e['event'] == 'paused']
    assert pauses and all('auto_resume_s' in p for p in pauses)
    assert all(p['auto_resume_s'] == 10.0 for p in pauses)                        # always the same countdown
    failed = [e for e in summary['events'] if e['event'] == 'destination_failed']
    assert all(e['attempts'] <= 3 for e in failed)                                 # bounded retries
    assert outcome == 'STOPPED' and summary['course_grid'] is not None


def test_an_operator_pause_waits_for_the_operator(tmp_path):
    _, summary, _, _ = roam(tmp_path, script=[(10, 'pause', None)], stop_after=60, cfg=cfg(auto_resume_s=5.0))
    n = names(summary)
    paused = first(summary, 'paused')
    assert paused['reason'] == 'operator pause' and 'auto_resume_s' not in paused
    assert 'auto_resumed' not in n and 'resumed' not in n


def test_map_reset_clears_the_grid_memory(tmp_path, monkeypatch):
    resets = []
    original_reset = CourseGrid.reset
    monkeypatch.setattr(CourseGrid, 'reset', lambda self, session=None: (resets.append(session),
                                                                         original_reset(self, session)))
    original = R.ExplorationRecord.observe_map
    state = {'done': False}

    def observe(self, area, now, session=None):
        if now > 20 and not state['done']:
            state['done'] = True
            self.reset(now, 'session changed')
            self.session = 'new-session'
            return 'session changed'
        return original(self, area, now, session)
    monkeypatch.setattr(R.ExplorationRecord, 'observe_map', observe)
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=cfg())
    assert 'map_reset' in names(summary) and 'new-session' in resets


def test_mission_events_are_written_to_a_json_file(tmp_path, monkeypatch):
    event = {'event_id': 'f1-object-0', 'type': 'observation', 'source': 'live_scene_understanding',
             'data': {'label': 'person', 'confidence': 0.8, 'location': {'frame_id': 'map', 'x': 1.0, 'y': 0.5}}}
    original = R.RoamSession._take_scene

    def take(self, now, pose):
        self.out_dir = tmp_path
        if now - self.t_roam > 5 and not getattr(self, '_fed', False):
            self._fed = True
            self.robot.mission = [(now, json.dumps(event))]
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)
    roam(tmp_path, stop_after=15, cfg=cfg())
    log = json.loads((tmp_path / 'mission_events.json').read_text())
    assert len(log) == 1 and log[0]['event']['value'] == 'person' and log[0]['event']['event_id'] == 'f1-object-0'
    assert set(log[0]) >= {'t_s', 'utc', 'robot', 'event'}


def test_overlay_shows_grid_alternatives_and_status(tmp_path, monkeypatch):
    frames = []
    monkeypatch.setattr(SimRoamRobot, 'publish_status_markers', lambda self, specs: frames.append(specs),
                        raising=False)
    roam(tmp_path, stop_after=60, cfg=cfg())
    last = frames[-1]
    ns = {m['ns'] for m in last}
    assert {'grid', 'status'} <= ns
    status = [m for m in last if m['ns'] == 'status']
    assert len(status) == 1 and status[0]['text']
    assert any('alternatives' in {m['ns'] for m in f} for f in frames)
    assert all(len(m['text']) <= 24 for f in frames for m in f if m['type'] == 'text' and m['ns'] != 'destination_labels')


def test_command_line_defaults_are_the_competition_ones():
    a = R.parse(['run', '--speed-limit', '0.5'])
    c = R.build_config(a, {'MAX_SPEED': '0.5'})
    assert c.decision == 'unified' and c.auto_resume_s == 15.0 and c.reselect
    assert c.weights == GS.DEFAULT_WEIGHTS
    c = R.build_config(R.parse(['run', '--speed-limit', '0.5', '--weights', 'grid=3', '--grid-cell', '1.5',
                                '--grid-origin', '1', '2', '--grid-heading-deg', '90', '--auto-resume', '0',
                                '--decision', 'sequence']), {'MAX_SPEED': '0.5'})
    assert c.weights['grid'] == 3 and c.grid_cell_m == 1.5 and c.grid_origin == (1.0, 2.0)
    assert c.grid_heading_deg == 90 and c.auto_resume_s == 0 and c.decision == 'sequence'
    assert math.isclose(GS.DEFAULT_WEIGHTS['info'], 1.0)


def test_physical_competition_start_selects_navigation_auto_mode_once():
    """Operator 2026-11-24: every competition_start --physical puts the robot in navigation (auto) mode."""
    import types
    from lite3_movement_tests.ros_io import NAVIGATION_MODE
    from lite3_movement_tests.runner import MODE_PHYSICAL, MODE_SIM
    sent = []
    robot = types.SimpleNamespace(can_actuate=True, now=lambda: 0.0,
                                  send_simple_command=lambda code: sent.append(code) or {'published': True})
    for mode, auto, expect in ((MODE_PHYSICAL, True, [NAVIGATION_MODE]), (MODE_PHYSICAL, False, []),
                               (MODE_SIM, True, [])):
        sent.clear()
        s = R.RoamSession(robot, R.RoamConfig(auto_mode=auto), mode, types.SimpleNamespace(), log=lambda *_: None)
        s._select_auto_mode()
        assert sent == expect
    a = R.parse(['run', '--speed-limit', '0.5', '--auto-mode', 'on'])
    assert R.build_config(a, {'MAX_SPEED': '0.5'}).auto_mode is True
    assert R.build_config(R.parse(['run', '--speed-limit', '0.5']), {'MAX_SPEED': '0.5'}).auto_mode is False
    script = (R.Path(__file__).resolve().parents[3] / 'operator' / 'competition_start.sh').read_text()
    assert '--physical --auto-mode on' in script


def pi_stairs(monkeypatch, after_s, distance_m=0.7, latency_s=1.5, period_s=1.7, obstacle='stairs', until_s=1e9,
              stamped=True):
    """The Pi's detector (Orbbec on the Pi) reporting confirmed stairs every period_s from after_s on. Each frame
    arrives latency_s after it was captured (capture stamp = arrival wall time - latency)."""
    original = R.RoamSession._take_scene
    state = {'next': None}

    def take(self, now, pose):
        if self.t_roam is not None and after_s <= now - self.t_roam <= until_s:
            if state['next'] is None or now >= state['next']:
                state['next'] = now + period_s
                wall = 1_000_000.0 + now
                reading = {'obstacle': obstacle, 'reason': 'confirmed', 'distance_m': distance_m,
                           'distance_reference': 'base_link_forward', 'heading_correction_deg': 0.0,
                           'capture_stamp_ns': int((wall - latency_s) * 1e9) if stamped else None}
                self.robot.pi_advisory = (now, wall, json.dumps(reading))
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)


def staircase(monkeypatch):
    from lite3_movement_tests import stairs as ST
    original_step = ST.StairRoutine.step

    def step(self, t, pitch, roll, travelled):
        p = -0.17 if 0.1 <= travelled < 0.5 else 0.17 if 0.75 <= travelled < 1.15 else 0.0
        return original_step(self, t, p, roll, travelled)
    monkeypatch.setattr(ST.StairRoutine, 'step', step)


def test_pi_stairs_stop_confirm_climb_in_ai_mode_and_return_to_auto_high_step(tmp_path, monkeypatch):
    """Operator 2026-11-24: detect a stair (Pi, Orbbec), switch to AI mode, climb over, switch back to navigation
    (auto) mode in the rug high-step gait with no AI. A reading captured before the stop is never acted on."""
    from lite3_movement_tests import ros_io as IO
    pi_stairs(monkeypatch, 6.0)
    staircase(monkeypatch)
    _, summary, robot, _ = roam(tmp_path, stop_after=90, cfg=cfg(gait='rug_hstep'))
    ev, n = summary['events'], names(summary)
    stop = first(summary, 'pi_obstacle_stop')
    confirmed = first(summary, 'pi_obstacle_confirmed')
    assert confirmed['by'] == 'capture stamp' and confirmed['stopped_s'] >= 1.5 + R.PI_STILL_MARGIN_S - 0.2
    start = first(summary, 'stairs_start')
    assert start['mode'] == 'ai' and start['trigger'] == 'advisory' and stop['t_s'] < start['t_s']
    assert any(e['event'] == 'advisory' and e.get('source') == 'pi' for e in ev)
    assert n.index('stairs_start') < n.index('stairs_done')
    cmds = [(s[1], s[2]) for s in robot.sent if s[1] in ('simple', 'gait')]
    enter = cmds.index(('simple', IO.AI_ENTER))
    leave = cmds.index(('simple', IO.AI_EXIT), enter)
    assert cmds[leave + 1] == ('simple', IO.NAVIGATION_MODE)                 # back to navigation (auto) mode
    assert cmds[leave + 2] == ('gait', 'rug_hstep')                          # in the rug high-step gait, AI off
    assert ('simple', IO.AI_ENTER) not in cmds[leave:leave + 3]

def test_pi_stairs_that_are_not_confirmed_standing_still_are_left(tmp_path, monkeypatch):
    pi_stairs(monkeypatch, 6.0, until_s=7.0)                  # one reading, then the Pi reports nothing
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=cfg())
    n = names(summary)
    assert 'pi_obstacle_stop' in n and 'pi_obstacle_unconfirmed' in n and 'stairs_start' not in n
    assert n.count('walk') >= 2                                # roaming carries on


def test_pi_readings_without_a_capture_stamp_wait_longer(tmp_path, monkeypatch):
    pi_stairs(monkeypatch, 6.0, stamped=False)
    _, summary, _, _ = roam(tmp_path, stop_after=30, cfg=cfg())
    c = first(summary, 'pi_obstacle_confirmed')
    assert c['by'] == 'arrival' and c['stopped_s'] >= R.PI_SETTLE_S - 0.1


def test_pi_readings_are_only_logged_in_log_mode_or_when_switched_off(tmp_path, monkeypatch):
    pi_stairs(monkeypatch, 6.0)
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=cfg(advisory='log'))
    n = names(summary)
    assert 'advisory' in n and 'pi_obstacle_stop' not in n and 'stairs_start' not in n
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=cfg(pi_advisory=False))
    assert 'pi_obstacle_stop' not in names(summary)


def test_after_the_climb_the_stairs_behind_do_not_stop_it_again(tmp_path, monkeypatch):
    pi_stairs(monkeypatch, 6.0)
    staircase(monkeypatch)
    _, summary, _, _ = roam(tmp_path, stop_after=60, cfg=cfg(gait='rug_hstep'))
    done = first(summary, 'stairs_done')['t_s']
    later = [e for e in summary['events'] if e['event'] == 'pi_obstacle_stop' and done < e['t_s'] < done + R.STAIR_COOLDOWN_S]
    assert not later


def test_auto_resume_accepts_a_robot_standing_and_stepping_in_place(tmp_path, monkeypatch):
    """Physical 2026-11-24: rug_hstep keeps stepping in place while standing, so 'joints still moving' refused every
    auto-resume. Standing with moving joints resumes; a tilted body still refuses, with the posture logged."""
    from lite3_movement_tests.runner import SequenceRunner
    no_goal_until(monkeypatch, 1e9)
    monkeypatch.setattr(R.RoamSession, '_grid_returns', lambda self, now, pose, relaxed=False: [])
    monkeypatch.setattr(R.RoamSession, '_backtrack', lambda self, now, pose: False)
    for reasons, resumes in ((['joints still moving'], True),
                             (['leg extensions disagree beyond spread limit', 'joints still moving'], True),
                             (['joints still moving', 'body tilt exceeds limit'], False)):
        monkeypatch.setattr(SequenceRunner, '_standing_precondition', lambda self, require_still=True, r=reasons: (
            False, {'posture': {'posture': 'standing', 'reasons': list(r), 'tilt_rad': None, 'extensions_m': {}}}))
        _, summary, _, _ = roam(tmp_path, stop_after=150, cfg=cfg(auto_resume_s=5.0))
        n = names(summary)
        assert 'paused' in n
        if resumes:
            assert 'auto_resumed' in n and 'resume_scan' in n
        else:
            assert 'auto_resumed' not in n and 'tilt' in first(summary, 'auto_resume_refused')['reason']
            assert first(summary, 'resume_refused_posture')['reasons'][-1] == 'body tilt exceeds limit'


def test_after_resuming_it_turns_round_to_scan_first(tmp_path):
    """Operator 2026-11-24: resuming after a pause rotates to scan the surroundings; the countdown is 15 s."""
    _, summary, _, _ = roam(tmp_path, script=[(10, 'pause', None), (14, 'resume', None)], stop_after=70, cfg=cfg())
    ev, n = summary['events'], names(summary)
    i = n.index('resume_scan')
    assert first(summary, 'resume_scan')['turns_deg'] == [R.LOOK_STEP_DEG] * 8
    after = [e for e in ev[i + 1:] if e['event'] in ('spin', 'walk')]
    assert after and after[0]['event'] == 'spin'                                  # turns before it walks on
    assert R.RoamConfig().auto_resume_s == 0.0
    assert R.build_config(R.parse(['run', '--speed-limit', '0.5']), {'MAX_SPEED': '0.5'}).auto_resume_s == 15.0


def test_a_nav2_path_through_a_wall_gap_is_cancelled(tmp_path, monkeypatch):
    import types
    import numpy as np
    from lite3_movement_tests import wall_gaps as WG
    s = R.RoamSession(types.SimpleNamespace(now=lambda: 10.0), cfg(), 'sim', types.SimpleNamespace(),
                      log=lambda *_: None)
    gaps = np.zeros((40, 40), dtype=bool)
    gaps[:, 20] = True                                                           # a closed gap line at x = 1.0
    s.wall_gaps = (gaps, (0.05, 0.0, 0.0))
    s.goal, s.goal_sent_at = {'kind': 'walk', 'x': 1.8, 'y': 0.5}, 5.0
    s.robot.nav_plan = (6.0, [(0.2, 0.5), (1.0, 0.5), (1.8, 0.5)])
    assert s._plan_through_wall(10.0)
    s._plan_check_t, s.robot.nav_plan = -1e9, (6.0, [(0.2, 0.5), (0.9, 0.5)])
    assert not s._plan_through_wall(10.0)
    s._plan_check_t, s.robot.nav_plan = -1e9, (4.0, [(0.2, 0.5), (1.8, 0.5)])   # plan older than this walk
    assert not s._plan_through_wall(10.0)
    assert WG.crosses(gaps, (0.05, 0.0, 0.0), [(0.2, 0.5), (1.8, 0.5)])


def test_blocked_turns_make_room_then_look_again(tmp_path, monkeypatch):
    """Physical 2026-11-24 14:20: next to a wall panel every turn ended "Collision Ahead" and nothing was worth
    walking to. After two blocked turns roam steps straight away (forward when clear) and looks round again."""
    from lite3_movement_tests.sim_model import FakeNavRobot

    def blocked(self):                                     # Nav2 spin: "Collision Ahead - Exiting Spin"
        self.goal_state['spun_rad'] = 0.02
        self.goal_state['result'] = 'ABORTED'
        self.spin = None
    monkeypatch.setattr(FakeNavRobot, '_spin_tick', blocked)
    no_goal_until(monkeypatch, 1e9)
    _, summary, robot, _ = roam(tmp_path, stop_after=60, cfg=cfg())
    n = names(summary)
    room = first(summary, 'make_room')
    assert room['way'] == 'forward' and room['step_m'] == R.MAKE_ROOM_M
    assert 'make_room_end' in n and n.index('make_room') < n.index('make_room_end')
    assert n.index('make_room') < n.index('scene_request') if 'scene_request' in n else True
    assert n.count('make_room') <= R.MAKE_ROOM_MAX * (n.count('stuck') or 1)
    after = n[n.index('make_room_end') + 1:]
    assert 'spin' in after                                  # looks round again from the new spot
    assert summary['stats']['make_room'] >= 1


def test_a_25_cm_block_on_its_pallet_is_crossed_not_treated_as_a_wall(tmp_path, monkeypatch):
    """Operator 2026-11-24: the block field is at most 25 cm from the ground; the terrain monitor measured 0.276 m
    (block + pallet) and roam refused it (limit was 0.25). Crossed in the high-step gait, never AI mode."""
    from lite3_movement_tests import ros_io as IO
    from test_m10 import _terrain_step
    _terrain_step(monkeypatch, 0.276)
    _, summary, robot, _ = roam(tmp_path, stop_after=20, cfg=cfg())
    step = first(summary, 'step_detected')
    assert abs(step['rise_m'] - 0.276) < 1e-6 and first(summary, 'stairs_start')['mode'] == 'hstep'
    assert IO.AI_ENTER not in [s[2] for s in robot.sent if s[1] == 'simple']


def test_a_step_crossing_never_touches_a_proximity_stop(tmp_path, monkeypatch):
    """Operator 2026-11-24: the terrain proximity stop is deleted; a crossing switches nothing on or off."""
    from lite3_movement_tests.sim_model import FakeNavRobot
    from test_m10 import _terrain_step
    calls = []
    monkeypatch.setattr(FakeNavRobot, 'set_terrain_proximity_stop', lambda self, on, timeout_s=2.0: calls.append(on)
                        or True, raising=False)
    _terrain_step(monkeypatch, 0.26)
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=cfg())
    assert 'stairs_start' in names(summary) and calls == []

def test_a_terrain_stop_reading_never_holds_the_dog(tmp_path, monkeypatch):
    """Operator 2026-11-24: the terrain proximity stop and roam's hold on it are deleted; Nav2 avoids obstacles."""
    from lite3_movement_tests.telemetry import Telemetry
    original = Telemetry.on_terrain

    def stopping(self, stamp, stop, reason, *a, **kw):
        return original(self, stamp, True, 'hazard 0.28 m ahead', *a, **kw)
    monkeypatch.setattr(Telemetry, 'on_terrain', stopping)
    for docs in (True, False):
        _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=cfg(docs_stops=docs))
        n = names(summary)
        assert 'hold' not in n and 'walk' in n

def test_centre_lanes_never_hold_the_dog(tmp_path):
    """Operator 2026-11-24 (run 14:05): with --centre-lane the dog off its cell's centre line was refused every lane
    and waited 10 s, again and again. Now a lane may start anywhere in the cell, and a refusal walks the goal."""
    c = cfg(grid_origin=(0.0, 0.35))                      # the dog starts 0.35 m off its cell's centre line
    c.centre_lane = True
    _, summary, _, _ = roam(tmp_path, stop_after=120, cfg=c)
    n = names(summary)
    assert 'centre_lane_hold' not in n
    assert n.count('walk') >= 3 and summary['stats']['distance_m'] > 3.0
    fallbacks = [e for e in summary['events'] if e['event'] == 'centre_lane_fallback']
    assert len(fallbacks) < n.count('walk')               # most walks use a lane; a refusal still walks


def scene_hold(monkeypatch, at_s, for_s=6.0, request_id='enc-1'):
    """nav_hold.py's message: hold from at_s for for_s (wall-clock expiry), then a release."""
    import time as _time
    original = R.RoamSession._scene_hold_tick
    state = {'sent': False}

    def tick(self, now, pose):
        if not state['sent'] and now - self.t_roam >= at_s:
            state['sent'] = True
            wall = _time.time_ns()
            self.robot.scene_hold_msg = json.dumps({'hold': True, 'request_id': request_id, 'encounter_ids': ['e1'],
                                                    'reason': 'scene_pending person', 'stamp_ns': wall,
                                                    'expires_wall_ns': wall + int(for_s * 1e9)})
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_scene_hold_tick', tick)


def test_a_pi_scene_hold_stops_squares_up_to_the_grid_then_carries_on(tmp_path, monkeypatch):
    """Operator 2026-11-24: on a Pi scene check the dog stops (not mid-stairs / curtain / roller), squares up to the
    1 x 1 m grid and waits for the result or the 6 s expiry, then roams on."""
    import time as _time
    scene_hold(monkeypatch, 4.0, for_s=1.5)              # 1.5 s of wall clock (the simulator runs faster)
    started = _time.time()
    _, summary, _, _ = roam(tmp_path, stop_after=60, cfg=cfg(grid_heading_deg=30.0))
    ev, n = summary['events'], names(summary)
    i, j = n.index('scene_hold'), n.index('scene_hold_end')
    assert first(summary, 'scene_hold')['request_id'] == 'enc-1'
    between = ev[i + 1:j]
    assert not [e for e in between if e['event'] == 'walk']                   # no new walk while holding
    squares = [e for e in between if e['event'] == 'spin']
    assert squares                                                              # turned square to the grid (+30 deg)
    assert 'walk' in n[j:]                                                      # carries on after the hold
    assert summary['stats']['scene_holds'] == 1 and _time.time() - started < 120


def test_an_expired_or_released_hold_is_clear():
    import time as _time
    from lite3_movement_tests.scene_hints import active_hold
    now = _time.time_ns()
    on = {'hold': True, 'request_id': 'r', 'stamp_ns': now, 'expires_wall_ns': now + 6_000_000_000}
    assert active_hold(json.dumps(on), now)['remaining_s'] == 6.0
    assert active_hold(json.dumps(on), now + 7_000_000_000) is None           # expired = clear
    assert active_hold(json.dumps(dict(on, hold=False)), now) is None
    assert active_hold(json.dumps(dict(on, stamp_ns=now - 60_000_000_000)), now) is None   # stale
    assert active_hold('garbage', now) is None


def test_stairs_square_up_to_the_grid_when_the_detector_is_close_to_it(tmp_path, monkeypatch):
    from test_m10 import _advise
    _advise(monkeypatch, 3.0, distance_m=0.7, heading_correction_deg=12.0)   # 12 deg off the grid axis
    _, summary, _, _ = roam(tmp_path, stop_after=30, cfg=cfg())
    n = names(summary)
    assert 'stairs_start' in n and 'stairs_align' not in n                     # already square with the grid
