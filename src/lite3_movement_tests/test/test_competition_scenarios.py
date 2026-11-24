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
    assert st['decisions'] >= 3 and st['goals_failed'] == 0 and st['distance_m'] > 8
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
    if len(pauses) > 1:
        assert pauses[1]['auto_resume_s'] > pauses[0]['auto_resume_s']              # backs off
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
