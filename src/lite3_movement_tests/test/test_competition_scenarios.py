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
    monkeypatch.setattr(R.RoamSession, '_grid_returns', lambda self, now, pose, relaxed=False: [])   # none left
    _, summary, _, _ = roam(tmp_path, stop_after=90, cfg=cfg())
    n = names(summary)
    assert 'spin' in n and n.index('spin') < n.index('scene_request')      # looked around first
    steps = [e['step'] for e in summary['events'] if e['event'] == 'stuck_step']
    assert steps.index('grid_return') < steps.index('make_room') if 'make_room' in steps else True   # item 4 order
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
    # operator 2026-11-25 (item 4): after looking round, the ladder heads straight back to an unfinished cell; the
    # scene request comes later, only when no unfinished cell is reachable
    back = [e for e in summary['events'] if e['event'] == 'stuck_step' and e['step'] == 'grid_return' and 'cell' in e]
    assert back
    looks = [e['t_s'] for e in summary['events'] if e['event'] == 'stuck_step' and e['step'] == 'look']
    assert not looks or looks[0] <= back[0]['t_s']
    walk = [e for e in summary['events'] if e['event'] == 'walk' and e['ends_at'] == 'grid_return']
    assert walk


def test_scene_requests_are_bounded(tmp_path, monkeypatch):
    sent = pi_link(monkeypatch, answer=None)
    no_goal_until(monkeypatch, 1e9)
    monkeypatch.setattr(R.RoamSession, '_grid_returns', lambda self, now, pose, relaxed=False: [])
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

    def step(self, t, pitch, roll, travelled, obstacle_m=None):
        p = -0.17 if 0.1 <= travelled < 0.5 else 0.17 if 0.75 <= travelled < 1.15 else 0.0
        ahead = 0.05 if travelled >= 1.3 else 1.0          # AI mode: front ultrasound past the bottom (wall)
        return original_step(self, t, p, roll, travelled, ahead)
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
    for reasons, tilt, resumes in ((['joints still moving'], None, True),
                                   (['leg extensions disagree beyond spread limit', 'joints still moving'], None, True),
                                   (['body tilt exceeds limit'], [0.0, -0.45], True),     # on a block (2026-11-24 c)
                                   (['joints still moving', 'body tilt exceeds limit'], [0.0, 0.7], False)):
        monkeypatch.setattr(SequenceRunner, '_standing_precondition', lambda self, require_still=True, r=reasons, t=tilt: (
            False, {'posture': {'posture': 'standing', 'reasons': list(r), 'tilt_rad': t, 'extensions_m': {}}}))
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
    monkeypatch.setattr(R.RoamSession, '_grid_returns', lambda self, now, pose, relaxed=False: [])
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
    _, summary, robot, _ = roam(tmp_path, stop_after=20, cfg=cfg(weights={'phase': 0.0}))   # walk at the block
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
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=cfg(weights={'phase': 0.0}))      # walk at the block
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
    assert n.count('walk') >= 3 and summary['stats']['distance_m'] > 2.5     # 3 lanes of ~1 m between two look-arounds
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
    _, summary, _, _ = roam(tmp_path, stop_after=30, cfg=cfg(weights={'phase': 0.0}))   # stairs: configured later
    n = names(summary)
    assert 'stairs_start' in n and 'stairs_align' not in n                     # already square with the grid



def test_a_lane_to_a_blocked_centre_aims_for_the_nearest_free_point(tmp_path, monkeypatch):
    """Operator 2026-11-24 fix 2: a blocked cell centre no longer refuses the lane."""
    original = R.RoamSession._goal_invalid

    def invalid(self, goal):
        if self.course is not None and 'centre_lane' in goal:
            cx, cy = self.course.centre(*self.course.cell(goal['x'], goal['y']))
            if math.hypot(goal['x'] - cx, goal['y'] - cy) < 0.05:
                return True                              # every exact cell centre is "on a block"
        return original(self, goal)
    monkeypatch.setattr(R.RoamSession, '_goal_invalid', invalid)
    c = cfg()
    c.centre_lane = True
    _, summary, _, _ = roam(tmp_path, stop_after=60, cfg=c)
    n = names(summary)
    assert 'centre_lane_nearest_free' in n
    near = first(summary, 'centre_lane_nearest_free')
    assert 0 < abs(near['offset_m']) <= 0.75 and n.count('walk') >= 2


def test_a_close_step_is_not_crossed_from_there_and_never_reversed_from(tmp_path, monkeypatch):
    """Operator 2026-11-24 (b): run 17:32 started a 0.29 m step 0.28 m away and aborted at 26 deg pitch. Crossings
    start from about STEP_START_M; reversing is refused (no rear sensing), so a close step waits for a farther look."""
    from test_m10 import _terrain_step
    _terrain_step(monkeypatch, 0.29, ahead=0.28, seconds=40.0)
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=cfg())
    n = names(summary)
    assert 'safety_stop' not in n
    close = first(summary, 'step_too_close')
    assert close['ahead_m'] == 0.28
    assert 'stairs_start' not in n or n.index('step_too_close') < n.index('stairs_start')


def test_a_step_met_from_0_45_m_is_crossed_at_once(tmp_path, monkeypatch):
    from test_m10 import _terrain_step
    _terrain_step(monkeypatch, 0.29, ahead=0.45, seconds=30.0)
    _, summary, _, _ = roam(tmp_path, stop_after=20, cfg=cfg())
    n = names(summary)
    assert 'step_too_close' not in n and 'stairs_start' in n


# ---------------------------------------------------------------------------------- LiDAR speckle (2026-11-24)
def test_speckle_is_small_blobs_only():
    import numpy as np
    from lite3_movement_tests import wall_gaps as WG
    data = np.zeros((40, 40), dtype=np.int16)
    data[5, 5] = 100                                        # one stray return
    data[3:5, 30] = 100                                     # a 2-cell small object barely in the scan slice
    data[30, 5:25] = 100                                    # a wall
    data[20, 5:30:4] = 100                                  # a wall SLAM mapped as dots 0.2 m apart
    data[20, 6:30:4] = 100
    m = WG.speckle_mask(data, 0.05)
    assert m[5, 5] and m[3, 30] and m[4, 30] and not m[30, 10:20].any() and not m[20].any()


def test_roam_reads_speckle_as_floor_and_its_costmap_veto_as_passable():
    from lite3_locomotion.frontiers import Grid
    s = R.RoamSession.__new__(R.RoamSession)
    s.stats, s.events = {}, []
    s.event = lambda name, **d: s.events.append(name)
    w = h = 40
    data = [0] * (w * h)
    data[20 * w + 20] = 100                                  # speckle at (1.025, 1.025)
    for i in range(0, 30):
        data[35 * w + i] = 100                              # wall along y = 1.775
    grid = Grid(w, h, 0.05, data, 0.0, 0.0)
    _, clean = s._despeckle((1.0, grid))
    assert clean.data[20 * w + 20] == 0 and clean.data[35 * w + 10] == 100
    assert s._speckle_zone(1.025, 1.025) and not s._speckle_zone(0.5, 1.775)
    cost = [0] * (w * h)
    for j in range(16, 25):
        for i in range(16, 25):
            cost[j * w + i] = 100                           # Nav2 inflation round the speckle
    for i in range(0, 30):
        cost[35 * w + i] = 100
    out = R.RoamSession._costmap_despeckled(Grid(w, h, 0.05, cost, 0.0, 0.0), s.speckle)
    assert out.data[20 * w + 20] < R.COSTMAP_VETO and out.data[35 * w + 10] == 100


def test_a_walk_blocked_by_speckle_becomes_a_probe_that_stops_5_cm_from_a_surface(tmp_path, monkeypatch):
    """Operator 2026-11-24: explore cells with minor speckling; walk until a surface 5 cm ahead, then roam on.
    Nav2 refuses the first walk (its costmap still holds the speckle): roam probes straight at the goal instead."""
    from lite3_movement_tests.sim_model import FakeNavRobot
    original = FakeNavRobot.nav_send_goal
    calls = []

    def send(self, x, y, yaw):
        calls.append((x, y))
        if len(calls) == 1:
            return False                                    # "goal/path in lethal cost"
        return original(self, x, y, yaw)
    monkeypatch.setattr(FakeNavRobot, 'nav_send_goal', send)
    evaluate = R.RoamSession._evaluate

    def first_goal_past_the_speckle(self, now, pose, note=True):
        if not calls:                                       # the unexplored cell behind the speckle
            goal = {'kind': 'walk', 'x': 2.4, 'y': 0.0, 'yaw': 0.0, 'ends_at': 'grid_return'}
            return GS.Decision(GS.Candidate(2.4, 0.0, 0.0, 'grid_return', goal=goal), 1.0, {}, [])
        return evaluate(self, now, pose, note)
    monkeypatch.setattr(R.RoamSession, '_evaluate', first_goal_past_the_speckle)
    _, summary, robot, _ = roam(tmp_path, stop_after=40, cfg=cfg(), map_obstacles=[(1.2, 0.0)])
    n = names(summary)
    assert 'safety_stop' not in n and summary['stats']['speckle_cells'] >= 1
    probe = first(summary, 'speckle_probe')
    end = first(summary, 'speckle_probe_end')
    assert n.index('speckle_probe') < n.index('speckle_probe_end') < n.index('walk')
    # walked up to the small object; the simulator has no live terrain monitor, so its stop is "no progress"/timeout
    assert end['how'] in ('surface 5 cm ahead', 'no progress', 'timeout') and end['walked_m'] > 0.3
    assert probe['length_m'] <= R.PROBE_MAX_M


def test_a_probe_stops_when_the_terrain_monitor_sees_a_surface_5_cm_ahead(tmp_path, monkeypatch):
    from lite3_movement_tests.sim_model import FakeNavRobot
    from lite3_movement_tests.telemetry import Telemetry
    sent = []
    original = FakeNavRobot.nav_send_goal
    monkeypatch.setattr(FakeNavRobot, 'nav_send_goal',
                        lambda self, x, y, yaw: (sent.append(1), len(sent) > 1 and original(self, x, y, yaw))[1])
    evaluate = R.RoamSession._evaluate

    def goal_past_the_speckle(self, now, pose, note=True):
        if not sent:
            goal = {'kind': 'walk', 'x': 2.4, 'y': 0.0, 'yaw': 0.0, 'ends_at': 'grid_return'}
            return GS.Decision(GS.Candidate(2.4, 0.0, 0.0, 'grid_return', goal=goal), 1.0, {}, [])
        return evaluate(self, now, pose, note)
    monkeypatch.setattr(R.RoamSession, '_evaluate', goal_past_the_speckle)
    on_terrain, t0 = Telemetry.on_terrain, {}

    def terrain(self, stamp, stop, reason, obstacle, *rest, **kw):
        t0.setdefault('t', stamp)
        if stamp > t0['t'] + 4.0:
            obstacle = R.ScanWalk.STANDOFF_M - 0.01                 # a surface 4 cm in front of the nose
        return on_terrain(self, stamp, stop, reason, obstacle, *rest, **kw)
    monkeypatch.setattr(Telemetry, 'on_terrain', terrain)
    _, summary, _, _ = roam(tmp_path, stop_after=20, cfg=cfg(), map_obstacles=[(1.2, 0.0)])
    end = first(summary, 'speckle_probe_end')
    assert end['how'] == 'surface 5 cm ahead' and 'safety_stop' not in names(summary)


def test_a_walk_that_barely_moves_is_ended():
    """Arena B 18:57: Nav2 crept 0.005 m/s for 60 s; every 0.10 m reset the 40 s no-progress check."""
    s = R.RoamSession.__new__(R.RoamSession)
    s.goal = {'kind': 'walk'}
    assert not s._crawling(0.0, (0.0, 0.0, 0.0))
    assert not s._crawling(10.0, (0.1, 0.0, 0.0))
    assert s._crawling(R.CRAWL_WINDOW_S + 0.1, (0.2, 0.0, 0.0))          # 0.2 m in 25 s
    assert not s._crawling(2 * R.CRAWL_WINDOW_S + 0.2, (0.6, 0.0, 0.0))   # 0.4 m: walking


def test_a_centre_lane_target_where_the_dog_already_stands_walks_the_real_goal():
    """Arena B 2026-11-24 19:17: the nearest free point of a blocked centre lay 0.2 m from the dog; Nav2 said
    "reached" after 0.00 m and the same lane was chosen 35 times (160 s standing still)."""
    s = R.RoamSession.__new__(R.RoamSession)
    s.course = CourseGrid(0.0, 0.0, 0.0)
    s.events, held = [], []
    s.event = lambda name, **d: s.events.append(name)
    s._goal_invalid = lambda g: abs(g['x'] - 1.0) < 1e-6 and abs(g['y']) < 1e-6     # the next centre is blocked
    s._reachable = lambda x, y: True
    s.course.return_point = lambda c, ok=None: (0.55, 0.0)                         # 0.25 m ahead of the dog
    s._centre_lane_hold = lambda now, why, goal=None, pose=None: held.append((why, goal))
    goal = {'kind': 'walk', 'x': 2.0, 'y': 0.0, 'yaw': 0.0}
    s._centre_lane_request(0.0, (0.3, 0.0, 0.0), goal)
    assert held and 'under' in held[0][0] and held[0][1] is goal          # walks the evaluator's own goal


def test_reached_without_moving_is_no_progress_and_resolves_no_stuck():
    s = R.RoamSession.__new__(R.RoamSession)
    s.events = []
    s.event = lambda name, **d: s.events.append((name, d))
    s.record, s.frontiers, s.course = None, None, None
    s.walk_start = (0.0, 0.0)
    s.goal = {'kind': 'walk', 'x': 0.2, 'y': 0.0, 'ends_at': 'patrol'}
    s.failures, s.ladder, s.ladder_pending = 0, {'name': 'look', 'step': 0}, None
    s.stats = {'goals_succeeded': 0, 'walks_to_obstacle': 0}
    s.cfg = cfg()
    s._end_walk(10.0, (0.01, 0.0, 0.0), 'reached')
    assert s.events[-1] == ('walk_end', {'how': 'reached_without_moving', 'walked_m': 0.01})
    assert s.stats['goals_succeeded'] == 0 and s.ladder is not None and s.failures == 1


# ---------------------------------------------------------------------------------- search / rooms (2026-11-25)
def _pocket():
    """A 1 x 1 m pocket at grid cell (2, 1): walls below, above and at the far end, open towards (1, 1)."""
    pts = []
    for k in range(23):                                   # 5 cm apart: the 0.1 m simulator grid leaves no holes
        x = 1.45 + 0.05 * k
        pts += [(x, 0.5), (x, 1.5)]
    for k in range(21):
        pts.append((2.5, 0.5 + 0.05 * k))
    return pts


def capture_link(monkeypatch, summary='kitchen', answer=True, first_status=None):
    """The Pi's capture side: each request is recorded and acked 1 s later (if answer); the final view of a room
    scan is followed by a room_scan_result naming the room (operator 2026-11-26)."""
    sent = []

    def publish(self, text):
        req = json.loads(text)
        sent.append(req)
        if answer:
            status = 'captured' if req.get('scan_id') else 'analysed'
            if first_status and len(sent) == 1:
                status = first_status                    # e.g. the first photo came out blurry
            self.capture_acks.append((self.now() + 1.0, json.dumps(
                {'id': req['id'], 'status': status, 'summary': summary,
                 'stored': True, 'scan_id': req.get('scan_id'), 'view_id': req.get('view_id')})))
            if req.get('scan_id') and req.get('final') and status == 'captured':
                self.room_scan_msgs.append((self.now() + 3.0, json.dumps(
                    {'event': 'room_scan_result', 'scan_id': req['scan_id'], 'room': {'type': summary},
                     'findings': [], 'qr': [], 'additional_view': {'needed': False}})))
    monkeypatch.setattr(SimRoamRobot, 'capture_acks', property(
        lambda self: self.__dict__.setdefault('_acks', [])), raising=False)
    monkeypatch.setattr(SimRoamRobot, 'room_scan_msgs', property(
        lambda self: self.__dict__.setdefault('_scans', [])), raising=False)
    monkeypatch.setattr(SimRoamRobot, 'publish_capture_request', publish, raising=False)
    return sent


def test_room_is_read_from_the_cell_in_front_and_never_entered(tmp_path, monkeypatch):
    """Operator 2026-11-26 (evening): the dog does NOT step into a room. It walks to the cell in front of the room's
    open side, faces in, stands still, and the Pi takes one forward photo (Claude + OpenCV QR); then it moves on."""
    sent = capture_link(monkeypatch)
    _, summary, _, session = roam(tmp_path, stop_after=150, cfg=cfg(geofence_m=5.0), map_obstacles=_pocket())
    n = names(summary)
    assert 'room_view_start' in n and 'room_view_done' in n and 'safety_stop' not in n
    done = [e for e in summary['events'] if e['event'] == 'room_view_done']
    assert done[0]['cell'] == [2, 1] and done[0]['room'] == 'kitchen' and done[0]['failure'] is None
    views = [r for r in sent if r.get('reason') == 'room view']
    assert [(r['view_id'], r['final'], tuple(r['cell'])) for r in views] == [('forward', True, (2, 1))]
    assert summary['course_grid']['rooms_read'].get('2,1') == 'kitchen'
    assert (2, 1) not in session.course.visited                 # never stepped into the room
    start = next(e for e in summary['events'] if e['event'] == 'room_view_start')
    assert start['t_s'] > 0 and 'walk' in n[n.index('room_view_done'):]      # moved on afterwards
    assert not any(r.get('reason') in ('room', 'room entry') for r in sent)  # no inside scan, no entry photo


def test_no_capture_ack_moves_off_after_the_timeout(tmp_path, monkeypatch):
    """A silent Pi: the forward photo is requested twice, then given up; the room is NOT read and NOT entered, the
    dog moves on and comes back to the front of it later (CAPTURE_RETRY_S)."""
    sent = capture_link(monkeypatch, answer=False)
    _, summary, _, session = roam(tmp_path, stop_after=150, cfg=cfg(geofence_m=5.0), map_obstacles=_pocket())
    done = [e for e in summary['events'] if e['event'] == 'room_view_done']
    assert done and done[0]['failure'] == 'photo failed: no ack, no ack'
    assert done[0]['waited_s'] >= R.ROOM_VIEW_ATTEMPTS * (R.ROOM_VIEW_SETTLE_S + R.ROOM_SCAN_VIEW_WAIT_S) - 1.0
    assert len([r for r in sent if r.get('scan_id') == done[0]['scan_id']]) == R.ROOM_VIEW_ATTEMPTS
    assert '2,1' not in summary['course_grid']['rooms_read'] and (2, 1) not in session.course.visited
    n = names(summary)
    assert 'walk' in n[n.index('room_view_done'):]


def test_blurry_room_photo_is_taken_again(tmp_path, monkeypatch):
    sent = capture_link(monkeypatch, first_status='blurry')
    _, summary, _, session = roam(tmp_path, stop_after=150, cfg=cfg(geofence_m=5.0), map_obstacles=_pocket())
    n = names(summary)
    retry = [e for e in summary['events'] if e['event'] == 'room_view_retry']
    assert retry and retry[0]['why'] == 'blurry'
    views = [r for r in sent if r.get('reason') == 'room view']
    assert [r['id'][-9:] for r in views] == ['forward-1', 'forward-2']
    done = [e for e in summary['events'] if e['event'] == 'room_view_done']
    assert done[0]['failure'] is None and done[0]['room'] == 'kitchen' and (2, 1) not in session.course.visited


def test_backtrack_only_when_a_room_is_unread():
    s = R.RoamSession.__new__(R.RoamSession)
    s.events = []
    s.event = lambda name, **d: s.events.append(name)
    s.cfg = cfg()
    s.course = CourseGrid(0.0, 0.0, 0.0)
    s._room_goals = lambda now, pose: []
    assert s._backtrack(0.0, (0.0, 0.0, 0.0)) is False and s.events == ['backtrack_skipped']


def test_a_reopened_passage_forgets_the_rejections_it_caused():
    from lite3_movement_tests.exploration_record import ExplorationRecord
    s = R.RoamSession.__new__(R.RoamSession)
    s.events, s.stats, s.goal = [], {}, None
    s.event = lambda name, **d: s.events.append((name, d))
    s.course = CourseGrid(0.0, 0.0, 0.0)
    s.record = ExplorationRecord('t', now=0.0)
    s.record.fail(1.0, 2.0, 0.0, 'walk ended (obstacle ahead)', 'terrain')
    key = frozenset(((1, 0), (2, 0)))
    s.course.edge_failures[key] = {'category': 'temporary', 'attempts': 1, 'until': 100.0}
    s.explorer = type('E', (), {'failed': [(1.9, 0.1, 100.0), (-3.0, 0.0, 100.0)]})()
    s.frontiers = None
    s._passage_reopened(5.0, [key])
    assert key not in s.course.edge_failures and not s.record.failures
    assert s.explorer.failed == [(-3.0, 0.0, 100.0)] and s.events[-1][0] == 'passage_reopened'
    assert any(v[3].endswith('reopened') for v in s._rejections.values())


def test_walk_goals_are_moved_to_the_middle_of_their_cell():
    from test_course_grid import grid_of
    s = R.RoamSession.__new__(R.RoamSession)
    s.course = CourseGrid(0.0, 0.0, 0.0)
    s.course.update(grid_of(free=[(-0.5, 3.5, -1.5, 1.5)]))
    s._reachable = lambda x, y: True
    s._in_course = lambda x, y: True
    s._raw_clear = lambda x, y, radius=0.3: True
    g = s._centred(0.0, (0.0, 0.0, 0.0), {'kind': 'walk', 'x': 2.3, 'y': 0.35, 'yaw': 0.0, 'ends_at': 'frontier'})
    assert math.dist((g['x'], g['y']), (2.0, 0.0)) <= 0.15 and g['centred'] == [2, 0]
    stairs = {'kind': 'walk', 'x': 2.3, 'y': 0.35, 'yaw': 0.0, 'ends_at': 'stairs'}
    assert s._centred(0.0, (0.0, 0.0, 0.0), stairs) is stairs


def test_no_in_place_turn_while_pressed_against_something(monkeypatch):
    """Operator 2026-11-25 (15:36 SGT flip): four look-around turns at 2-3 cm clearance, the fifth flipped the dog.
    Narrowed: only optional look-around turns are skipped, only while touching (clearance < 5 cm)."""
    s = R.RoamSession.__new__(R.RoamSession)
    s.events, s.stats, s.look, s.record = [], {'spins': 0}, [0.7, 0.7], None
    s.map = (0.0, None)
    s.event = lambda name, **d: s.events.append((name, d))
    monkeypatch.setattr(R, 'known_area_m2', lambda grid: 0.0)
    spins, clearance = [], [0.02]
    s.robot = type('Rb', (), {'nav_spin': lambda self, a, t: spins.append(a) or True,
                              'telemetry': type('T', (), {'clearance': type('C', (), {
                                  'latest': staticmethod(lambda: (0.0, clearance[0]))})()})()})()
    s._start_spin(1.0, (0.0, 0.0, 0.0), 0.7, reason='look-around: unseen floor to the side')
    assert spins == [] and s.look == [] and s.events[-1][0] == 'turn_skipped'
    assert 'clearance 0.02' in s.events[-1][1]['pressed']
    s._start_spin(1.0, (0.0, 0.0, 0.0), 0.7, reason='face room 2,1')          # not optional: sent to Nav2
    assert spins == [0.7]
    clearance[0] = 0.08                                                     # close but not touching: turns
    s._start_spin(2.0, (0.0, 0.0, 0.0), 0.7, reason='look-around: unseen floor to the side')
    assert spins == [0.7, 0.7]


def test_turn_checks_use_the_raw_map_with_small_blocks():
    """Speckle-as-floor is for goals only: turning near a small block must still see it (07:36 flip)."""
    from test_course_grid import grid_of
    s = R.RoamSession.__new__(R.RoamSession)
    raw = grid_of(free=[(-1.0, 2.0, -1.0, 1.0)], occupied=[(0.4, 0.45, 0.0, 0.05)])     # one small block
    s._map_raw = (1.0, raw)
    s.map = (1.0, grid_of(free=[(-1.0, 2.0, -1.0, 1.0)]))                            # cleaned view
    s.robot = type('Rb', (), {'now': lambda self: 0.0})()
    grid, cost = s._raw_maps()
    assert grid is raw and cost is None
    pts = R.ScanWalk.known_obstacles(grid, cost, (0.0, 0.0), 1.0)
    assert any(math.dist(p, (0.42, 0.02)) < 0.1 for p in pts)


def test_a_room_goal_ends_facing_the_way_it_walked_then_turns_slowly_to_face_in():
    """Operator 2026-11-25 (17:10 / 17:14 SGT flips): a room goal ending 'face back into the room' made Nav2's walking
    controller turn the dog ~180 deg at up to 1.5 rad/s on a block. The walk now ends in its travel direction and the
    dog faces the room with an in-place turn (Nav2 Spin, 0.5 rad/s) once there."""
    from test_course_grid import _room_map
    s = R.RoamSession.__new__(R.RoamSession)
    s.course = CourseGrid(0.0, 0.0, 0.0)
    s.course.update(_room_map())
    for corridor in ((0, 0), (1, 0)):                       # 2 walls (above, below): rooms too since 2026-11-26
        s.course.mark_room_read(corridor, 'corridor')
    s._reachable = lambda x, y: True
    s._in_course = lambda x, y: True
    s._raw_clear = lambda x, y, radius=0.3: True
    pose = (-0.2, 0.0, 0.0)                                  # walking towards the room along +x
    goal = s._room_goals(0.0, pose)[0]
    travel = math.atan2(goal['y'] - pose[1], goal['x'] - pose[0])
    # Operator 2026-11-26 (evening): the goal is the cell IN FRONT of the room (never inside). The walk still ends
    # facing its travel direction; face_yaw (into the room) is reached afterwards with a slow in-place turn.
    assert abs(goal['yaw'] - travel) < 1e-9 and goal['cell'] == [2, 0] and goal['room_view']
    assert s.course.cell(goal['x'], goal['y']) != (2, 0)
    cx, cy = s.course.centre(2, 0)
    assert abs(math.remainder(goal['face_yaw'] - math.atan2(cy - goal['y'], cx - goal['x']), math.tau)) < 1e-9
    behind = (3.0, 0.0, math.pi)                             # coming from the far side: still ends facing its walk
    g2 = s._room_goals(0.0, behind)[0]
    assert abs(math.cos(g2['yaw'] - math.atan2(g2['y'] - behind[1], g2['x'] - behind[0])) - 1.0) < 1e-9


def test_a_terrain_reading_inside_the_body_never_ends_a_walk(tmp_path, monkeypatch):
    """Operator 2026-11-25: roam's walking-standoff cancel is removed (Nav2 alone avoids obstacles). Tilted on a block
    the terrain monitor reported 0.025 m ahead of the centre - inside the dog - and open walks were ended."""
    from lite3_movement_tests.telemetry import Telemetry
    on_terrain = Telemetry.on_terrain

    def terrain(self, stamp, stop, reason, obstacle, *rest, **kw):
        return on_terrain(self, stamp, stop, reason, 0.025, *rest, **kw)
    monkeypatch.setattr(Telemetry, 'on_terrain', terrain)
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=cfg())
    reasons = [str(e.get('how', '')) + str(e.get('reason', '')) for e in summary['events']
               if e['event'] in ('walk_end', 'walk_failed')]
    assert not any('walking standoff' in r for r in reasons)
    assert summary['stats']['distance_m'] > 0.8


def test_nav2_leaving_the_centre_lane_keeps_walking(tmp_path, monkeypatch):
    """Operator 2026-11-25 fix 2 (18:04 SGT): 'Nav2 replanned outside the centre lane' cancelled open walks and
    marked them failed (one avoided 120 s). Now the walk goes on."""
    state = {'n': 0}

    def changed(self, now):
        if self.goal and 'centre_lane' in self.goal and state['n'] == 0:
            state['n'] += 1
            return True
        return False
    monkeypatch.setattr(R.RoamSession, '_centre_lane_changed', changed)
    c = cfg()
    c.centre_lane = True
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=c)
    n = names(summary)
    assert 'centre_lane_left' in n
    assert not any('centre lane' in str(e.get('reason', '')) for e in summary['events'] if e['event'] == 'walk_failed')


def test_after_climbing_a_block_only_straight_on_goals_are_chosen():
    """Operator 2026-11-25 fix 3 (18:04 SGT: up a 0.28 m block, then sent off it at -124 deg, tilt stop)."""
    s = R.RoamSession.__new__(R.RoamSession)
    s.events = []
    s.event = lambda name, **d: s.events.append(name)
    s.on_block = {'t': 0.0, 'x': 0.0, 'y': 0.0, 'heading': 0.0}
    import types
    tilt = [(0.0, 0.0)]
    s.robot = types.SimpleNamespace(telemetry=types.SimpleNamespace(current_tilt=lambda: tilt[0]))
    ahead = GS.Candidate(1.0, 0.1, 0.0, 'grid_return')
    sideways = GS.Candidate(-0.5, -0.8, 0.0, 'room')
    assert s._on_block_filter(1.0, (0.0, 0.0, 0.0), [ahead, sideways]) == [ahead]
    assert s._on_block_filter(1.0, (0.0, 0.0, 0.0), [sideways]) == [sideways]          # nothing straight on: as before
    tilt[0] = (0.37, 0.0)                                   # 21 deg: still half on it (operator 2026-11-26, 18:14 run)
    assert s._on_block_filter(99.0, (0.8, 0.0, 0.0), [ahead, sideways]) == [ahead] and s.on_block is not None
    tilt[0] = (0.0, 0.0)
    assert s._on_block_filter(2.0, (0.8, 0.0, 0.0), [ahead, sideways]) == [ahead, sideways]   # walked off it
    assert s.on_block is None and s.events[-1] == 'off_block'


def test_no_camera_stop_while_tilted_or_a_step_is_just_ahead():
    """Arena B 2026-11-25 23:41 SGT: the capture stop cut a walk partway up a 0.3 m block; pitch rose to 0.565 rad."""
    import types
    s = R.RoamSession.__new__(R.RoamSession)
    s.cfg = cfg()
    tilt, step = [(0.0, 0.25)], [None]
    s.robot = types.SimpleNamespace(telemetry=types.SimpleNamespace(current_tilt=lambda: tilt[0]))
    s._terrain_now = lambda now: step[0]
    assert 'tilted' in s._capture_uneven(0.0)
    tilt[0] = (0.0, 0.02)
    step[0] = types.SimpleNamespace(step_ahead_m=0.5, step_rise_m=0.28)
    assert 'step' in s._capture_uneven(0.0)
    step[0] = types.SimpleNamespace(step_ahead_m=1.5, step_rise_m=0.28)
    assert s._capture_uneven(0.0) is None                       # level, block well ahead: stop for the picture


def test_roam_works_on_a_course_sized_window_of_a_huge_slam_map():
    """Heartbeat fixes 2-3 (2026-11-26): SLAM mapped 41 x 43 m (700k cells) and every map step scaled with it."""
    from lite3_locomotion.frontiers import Grid
    from lite3_movement_tests.exploration_record import ExplorationRecord
    n = 600                                                       # 30 x 30 m at 5 cm
    data = [0] * (n * n)
    data[300 * n + 300] = 100                                     # a speckle near the course
    g = Grid(n, n, 0.05, data, -15.0, -15.0)
    s = R.RoamSession.__new__(R.RoamSession)
    s.stats, s.events, s.channel = {}, [], None
    s.event = lambda *a, **k: None
    s.cfg = cfg(geofence_m=5.0)
    s.anchor, s.record = (0.0, 0.0), ExplorationRecord('w', now=0.0)
    s.record.add_breadcrumb(1.0, 0.0)
    _, view = s._despeckle((1.0, g))
    assert view.width * view.height < n * n / 4                     # course-sized, not the whole map
    assert view.origin_x >= -6.1 and view.origin_x + view.width * 0.05 <= 6.1
    x, y = view.world(view.cell(0.025, 0.025))
    assert view.value(view.cell(0.025, 0.025)) == 0                 # the speckle is floor in the view
    assert abs(x - 0.025) < 0.03 and abs(y - 0.025) < 0.03          # same world coordinates as the SLAM map


def test_no_space_to_turn_walks_back_until_there_is_room(tmp_path, monkeypatch):
    """Operator 2026-11-26: "if it has no space to turn, walk backwards till there is space"."""
    state = {'t0': None}
    real_room = R.RoamSession._room_to_turn

    def pressed(self, now):
        state['t0'] = state['t0'] if state['t0'] is not None else now
        return 'clearance 0.02 m' if now - state['t0'] < 8.0 else None
    monkeypatch.setattr(R.RoamSession, '_pressed', pressed)
    monkeypatch.setattr(R.RoamSession, '_room_to_turn', lambda self, pose: real_room(self, pose))
    no_goal_until(monkeypatch, 1e9)                          # only turning: look / search turns
    _, summary, robot, _ = roam(tmp_path, stop_after=40, cfg=cfg())
    n = names(summary)
    assert 'safety_stop' not in n and 'back_for_room' in n
    start, end = first(summary, 'back_for_room'), first(summary, 'back_for_room_end')
    assert start['max_m'] <= R.BACK_FOR_ROOM_MAX_M and end['how'] in ('room to turn', 'done', 'obstacle behind')
    assert n.index('back_for_room') < n.index('back_for_room_end') < len(n) - 1
    assert 'spin' in n[n.index('back_for_room_end'):]          # turns once there is room


def test_no_back_off_with_something_close_behind():
    import types
    s = R.RoamSession.__new__(R.RoamSession)
    s.events, s.stats, s.nudge, s.stairs, s.goal = [], {}, None, None, None
    s.event = lambda name, **d: s.events.append(name)
    s.robot = types.SimpleNamespace(can_actuate=True, telemetry=types.SimpleNamespace(
        odom=types.SimpleNamespace(latest=lambda: (0.0, types.SimpleNamespace(x=0.0, y=0.0)))))
    s._rear_obstacle = lambda: 0.3
    assert s._back_for_room(0.0, (0.0, 0.0, 0.0), 'test') is False and s.events == ['back_for_room_impossible']
    s._rear_obstacle = lambda: 2.0
    assert s._back_for_room(1.0, (0.0, 0.0, 0.0), 'test') and s.nudge['sign'] == -1
    assert s.nudge['length'] == R.BACK_FOR_ROOM_MAX_M


def test_roaming_continues_while_the_lidar_rate_is_low_but_at_least_1_hz(tmp_path, monkeypatch):
    """Operator 2026-11-26: passing a curtain drops the LiDAR rate; keep going down to 1 Hz (was 6 Hz)."""
    from lite3_movement_tests.telemetry import Telemetry
    real_init = Telemetry.__init__

    def init(self, *a, **k):
        real_init(self, *a, **k)
        add, n = self.raw_cloud.add, {'k': 0}

        def thin(stamp, value):                          # every 5th cloud: 2 Hz instead of 10 Hz
            n['k'] += 1
            if n['k'] % 5 == 0:
                add(stamp, value)
        self.raw_cloud.add = thin
    monkeypatch.setattr(Telemetry, 'init_marker', True, raising=False)
    monkeypatch.setattr(Telemetry, '__init__', init)
    _, summary, _, _ = roam(tmp_path, stop_after=30, cfg=cfg())
    assert 'safety_stop' not in names(summary) and summary['roaming_s'] >= 25      # roamed the whole time at 2 Hz
    assert R.HARD_MIN_LIDAR_HZ == 1.0


def test_a_short_lidar_dropout_under_a_curtain_does_not_stop_roaming():
    """Operator 2026-11-26: relax both other LiDAR checks for curtains (stale 1 -> 3 s; nearly empty 1 -> 3 s,
    never during a curtain crossing)."""
    assert R.ROAM_RAW_LIDAR_STALE_S == 3.0 and R.RAW_LIDAR_EMPTY_S == 3.0
    assert R.WatchdogLogic.RAW_LIDAR_AGE_S > R.ROAM_RAW_LIDAR_STALE_S


def test_the_nuc_decides_standing_still_before_any_photo():
    """Operator 2026-11-26: "this should be implemented in the NUC, not the Pi" - roam measures stillness (leg odometry
    and IMU) and only then asks for the photo, marked still=true; the Pi no longer has to guess from commands."""
    import types
    from lite3_movement_tests.telemetry import Telemetry
    t = Telemetry()
    s = R.RoamSession.__new__(R.RoamSession)
    s.robot = types.SimpleNamespace(telemetry=t)
    for k in range(10):                                      # walking until 1.0 s, then standing
        ts = 0.1 * k
        t.on_odom(ts, 0, 0, 0.3, 0, 0, 0, 1, 0.2 if ts < 1.0 else 0.0, 0, 0)
    for k in range(10, 30):
        t.on_odom(0.1 * k, 0, 0, 0.3, 0, 0, 0, 1, 0.0, 0, 0)
    assert not s._still_enough(1.3)                          # only 0.3 s still
    assert s._still_enough(2.5) and s._still_fields(2.5)['still'] is True
    t.on_odom(2.6, 0, 0, 0.3, 0, 0, 0, 1, 0.0, 0, 0.3)       # turning again
    assert not s._still_enough(2.6) and s._still_fields(2.6) == {'still': False}


def test_no_walk_goal_ever_ends_inside_a_known_room():
    """Operator 2026-11-26: the dog never steps into a room (any goal source); it looks in from the cell in front."""
    from test_course_grid import _room_map
    s = R.RoamSession.__new__(R.RoamSession)
    s.course = CourseGrid(0.0, 0.0, 0.0)
    s.course.update(_room_map())                              # (2, 0) is a room, open towards (1, 0)
    s.events, s.failed = [], []
    s.event = lambda name, **d: s.events.append(name)
    s._walk_failed = lambda now, why, goal=None: s.failed.append(why)
    inside = {'kind': 'walk', 'x': 2.0, 'y': 0.0, 'yaw': 0.0, 'ends_at': 'straight_on'}   # (1, 0): corridor = path
    front = {'kind': 'walk', 'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'ends_at': 'room'}
    assert s._in_room(inside) and not s._in_room(front)
    assert not s._in_room(dict(inside, ends_at='stairs'))     # stairs / scene goals are left alone
    s._start_walk(0.0, (0.0, 0.0, 0.0), inside)
    assert s.events == ['walk_refused'] and s.failed == ['goal inside a room']


def test_the_terminal_says_what_the_dog_is_doing_in_plain_words(tmp_path, monkeypatch):
    """Operator 2026-11-26: print in simple terms what is happening (scene understanding, walking to goal,
    rethinking, climbing stairs, entering curtains) - once per change, not every second."""
    said = []
    original = R.RoamSession._say_activity

    def say(self, now):
        self.log = said.append
        return original(self, now)
    monkeypatch.setattr(R.RoamSession, '_say_activity', say)
    _, summary, _, _ = roam(tmp_path, stop_after=60, cfg=cfg())
    lines = [l for l in said if 'NOW:' in l]
    assert lines and all(a != b for a, b in zip(lines, lines[1:]))          # only on change
    text = ' '.join(lines)
    assert 'Walking to' in text and ('Rethinking' in text or 'Looking around' in text)


def test_plain_activity_words_for_the_special_states():
    s = R.RoamSession.__new__(R.RoamSession)
    s.state, s.reason, s.auto_resume_at = 'ROAMING', None, None
    for k in ('stairs', 'crossing', 'nudge', 'pi_gate', 'scene_hold', 'scene_wait', 'scene_task', 'capture_task',
              'room_task', 'hold_since', 'goal', 'ladder', 'record'):
        setattr(s, k, None)
    s.look = []
    assert s._plain_activity(0.0).startswith('Rethinking')
    s.crossing = {'probe': False}
    assert s._plain_activity(0.0) == 'Entering a curtain'
    s.crossing = None
    s.stairs, s.stairs_kind = (type('Rt', (), {'direction': 'up'})(), None, None), 'stairs'
    assert s._plain_activity(0.0) == 'Climbing stairs (up, AI mode)'
    s.stairs = None
    s.scene_hold = {'id': 'x'}
    assert s._plain_activity(0.0).startswith('Scene understanding')
    s.scene_hold = None
    s.goal = {'kind': 'walk', 'ends_at': 'grid_return'}
    assert s._plain_activity(0.0) == 'Walking to an unexplored cell'
    s.state, s.auto_resume_at, s.goal = 'PAUSED', 15.0, None
    assert s._plain_activity(0.0) == 'Paused - resumes by itself in 15 s'
