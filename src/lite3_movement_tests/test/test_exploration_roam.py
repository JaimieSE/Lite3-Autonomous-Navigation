"""Exploration record wired into roam (simulator, full roam loop): avoidance, retries, inspections, overlay, reset."""
import json
import math

import pytest

from lite3_movement_tests import roam as R
from lite3_movement_tests.roam_sim import SimRoamRobot
from test_m10 import _grid, roam


@pytest.fixture
def overlay(monkeypatch):
    """Capture every overlay the simulated robot is asked to publish."""
    monkeypatch.setattr(SimRoamRobot, 'publish_status_markers',
                        lambda self, specs: self.__dict__.setdefault('overlay', []).append(specs), raising=False)


def feed(monkeypatch, when, inbox, payload, times=1):
    """Put payload (JSON) into the robot inbox `inbox` once when(session, now) first holds (times copies)."""
    original = R.RoamSession._take_scene
    state = {'sent': False}

    def take(self, now, pose):
        if not state['sent'] and when(self, now):
            state['sent'] = True
            box = self.robot.__dict__.setdefault(inbox, [])
            box.extend([(self.robot.now(), json.dumps(payload))] * times)
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)


def cfg(**kw):
    return R.RoamConfig(strategy='scan_walk', stop_at_exit=False, start_mode='normal', **kw)


def names(summary):
    return [e['event'] for e in summary['events']]


def test_choose_skips_a_destination_the_record_avoids():
    grid = _grid(free=[(-0.5, 3.0, -0.7, 0.7)], occupied=[(3.0, 3.1, -1, 1)])
    w = R.ScanWalk(cfg(geofence_m=6.0), (0.0, 0.0))
    w.record = R.ExplorationRecord('t')
    first = w.choose(grid, (0.0, 0.0, 0.0), 0.0)
    assert first is not None
    w.record.fail(0.0, first['x'], first['y'], 'walk ended after 0.02 m (obstacle ahead (walking standoff))')
    again = w.choose(grid, (0.0, 0.0, 0.0), 1.0)
    assert again is None or math.hypot(again['x'] - first['x'], again['y'] - first['y']) > 0.4


def test_failed_walks_are_recorded_and_not_repeated(tmp_path, monkeypatch, overlay):
    """Every walk is cut short at once (hard terrain): each failed spot is recorded as 'terrain' (operator
    2026-11-24: uneven ground is not inaccessible) and not re-chosen before its retry wait."""
    original = R.RoamSession._end_walk

    def cut_short(self, now, pose, how):
        if how == 'reached':
            self.walk_start = pose                       # walked nothing: counts as a failure
            how = 'obstacle ahead (walking standoff)'
        return original(self, now, pose, how)
    monkeypatch.setattr(R.RoamSession, '_end_walk', cut_short)
    _, summary, robot, _ = roam(tmp_path, cfg=cfg(), stop_after=150)
    failed = [e for e in summary['events'] if e['event'] == 'destination_failed']
    assert failed and all(e['category'] == 'terrain' for e in failed)
    from lite3_movement_tests.exploration_record import TERRAIN_RETRY_S
    walks = [(e['t_s'], e['x'], e['y']) for e in summary['events'] if e['event'] == 'walk']
    repeats = [(a, b) for i, a in enumerate(walks) for b in walks[i + 1:]
               if math.hypot(a[1] - b[1], a[2] - b[2]) < 0.3 and b[0] - a[0] < TERRAIN_RETRY_S]
    assert not repeats, f'same destination re-chosen: {repeats}'
    assert summary['exploration']['failures'] >= 1
    assert any(e['event'] == 'paused' for e in summary['events']) or len(failed) < summary['config']['max_goal_failures']


def test_temporary_failure_is_retried_after_the_delay(tmp_path, monkeypatch, overlay):
    calls = {'n': 0}
    real = SimRoamRobot.nav_send_goal

    def flaky(self, x, y, yaw):
        calls['n'] += 1
        return False if calls['n'] == 1 else real(self, x, y, yaw)          # Nav2 server hiccup once
    monkeypatch.setattr(SimRoamRobot, 'nav_send_goal', flaky)
    _, summary, _, _ = roam(tmp_path, cfg=cfg(), stop_after=60)
    ev = summary['events']
    first = next(e for e in ev if e['event'] == 'destination_failed')
    assert first['category'] == 'temporary' and 'retry after' in first['retry']
    assert any(e['event'] == 'walk' for e in ev)                             # exploration went on


REQ = {'request_id': 'insp-1', 'mission_ids': ['M03'], 'evidence_ref': 'frame-42',
       'viewing_pose': {'x': 0.9, 'y': 0.0, 'yaw': math.pi / 2}, 'priority': 2}


def test_inspection_handoff_confirmed_then_exploration_resumes(tmp_path, monkeypatch, overlay):
    feed(monkeypatch, lambda s, now: s.t_roam is not None and now - s.t_roam >= 2.0, 'inspection_requests', REQ)
    original = R.RoamSession._inspection_tick

    def confirm_on_arrival(self, now, pose):
        out = original(self, now, pose)
        insp = self.record.inspections.get(self.record.active_inspection or '')
        if insp and insp['arrived'] is not None and now - insp['arrived'] > 1.0:
            self.robot.__dict__.setdefault('camera_confirmations', []).append(
                (self.robot.now(), json.dumps({'usable': True, 'request_id': 'insp-1'})))
        return out
    monkeypatch.setattr(R.RoamSession, '_inspection_tick', confirm_on_arrival)
    _, summary, robot, _ = roam(tmp_path, cfg=cfg(), stop_after=160)
    n = names(summary)
    assert n.index('inspection_request') < n.index('inspection_start') < n.index('inspection_arrived')
    conf = next(e for e in summary['events'] if e['event'] == 'camera_confirmation')
    assert conf['applied_to'] == 'inspection'
    assert summary['exploration']['inspections']['INSPECTED'] == 1
    after = n[n.index('camera_confirmation') + 1:]
    assert 'walk' in after or 'spin' in after                                # ordinary exploration resumed
    walk = next(e for e in summary['events'] if e['event'] == 'walk' and e['ends_at'] == 'inspection')
    assert abs(walk['x'] - 0.9) < 0.05


def test_inspection_without_camera_confirmation_times_out_deferred(tmp_path, monkeypatch, overlay):
    feed(monkeypatch, lambda s, now: s.t_roam is not None and now - s.t_roam >= 2.0, 'inspection_requests', REQ)
    _, summary, _, _ = roam(tmp_path, cfg=cfg(), stop_after=140)
    end = next(e for e in summary['events'] if e['event'] == 'inspection_end')
    assert end['state'] == 'DEFERRED' and 'no usable-camera confirmation' in end['reason']
    assert summary['exploration']['inspections']['INSPECTED'] == 0          # never INSPECTED without the camera
    n = names(summary)
    after = n[n.index('inspection_end') + 1:]
    assert any(x in after for x in ('walk', 'spin', 'look_around'))         # ordinary exploration resumed
    assert after[:1] != ['inspection_start']                                 # not the same inspection at once


def test_duplicate_inspection_requests_make_one_record(tmp_path, monkeypatch, overlay):
    feed(monkeypatch, lambda s, now: s.t_roam is not None and now - s.t_roam >= 2.0, 'inspection_requests', REQ,
         times=3)
    _, summary, _, _ = roam(tmp_path, cfg=cfg(), stop_after=20)
    results = [e['result'] for e in summary['events'] if e['event'] == 'inspection_request']
    assert results == ['added', 'duplicate', 'duplicate']
    assert sum(summary['exploration']['inspections'].values()) == 1


def test_overlay_published_and_drawn_from_the_record(tmp_path, overlay):
    _, summary, robot, _ = roam(tmp_path, cfg=cfg(), stop_after=40)
    published = robot.__dict__.get('overlay', [])
    assert len(published) >= 10                                              # ~1 Hz and on change
    last = published[-1]
    ns = {m['ns'] for m in last}
    assert 'breadcrumbs' in ns and 'destination' in ns
    assert len(next(m for m in last if m['ns'] == 'breadcrumbs')['points']) == summary['exploration']['breadcrumbs']


def test_marker_array_starts_with_deleteall_and_converts_every_spec():
    from builtin_interfaces.msg import Time
    from visualization_msgs.msg import Marker
    from lite3_movement_tests.exploration_record import ExplorationRecord, build_markers
    from lite3_movement_tests.exploration_ros import markers_to_msg
    r = ExplorationRecord('t')
    r.set_destination(0.0, {'x': 1.0, 'y': 0.5, 'yaw': 0.3}, 'walk', 'frontier')
    r.add_breadcrumb(0.0, 0.0); r.add_breadcrumb(1.0, 0.0)
    r.note_decision((0, 0, 0), [0.0, math.pi / 2])
    r.fail(0.0, 2.0, 2.0, 'Nav2 ABORTED')
    r.add_inspection(0.0, dict(REQ))
    specs = build_markers(r, labels=True)
    msg = markers_to_msg(specs, Time(sec=1))
    assert msg.markers[0].action == Marker.DELETEALL                        # obsolete markers removed
    assert len(msg.markers) == len(specs) + 1
    assert all(m.header.frame_id == 'map' for m in msg.markers)
    texts = [m.text for m in msg.markers if m.type == Marker.TEXT_VIEW_FACING]
    assert any('frontier' in t for t in texts) and any(t.startswith('#1 M03') for t in texts)
    assert any(t.startswith('J1') for t in texts) and any('x1 terrain' in t for t in texts)
    r.reset(1.0, 'map reset')
    cleared = markers_to_msg(build_markers(r), Time(sec=2))
    assert cleared.markers[0].action == Marker.DELETEALL
    assert {m.ns for m in cleared.markers[1:]} <= {'destination'}


def test_map_reset_mid_run_invalidates_the_record(tmp_path, monkeypatch, overlay):
    """slam_toolbox map cleared mid-run: path, junctions, failures and camera coverage start again."""
    real = SimRoamRobot.map_grid

    def shrinking(self):
        item = real(self)
        started = getattr(self, 'roam_started', None)
        if item is not None and started is not None and getattr(self, 't', 0) > started + 25:
            t, g = item
            from lite3_locomotion.frontiers import Grid
            small = Grid(20, 20, g.resolution, [0] * 400, -0.5, -0.5)   # 1 m2: "the map was reset"
            return (t + 1000.0, small)
        return item
    monkeypatch.setattr(SimRoamRobot, 'map_grid', shrinking)
    _, summary, robot, _ = roam(tmp_path, cfg=cfg(), stop_after=45)
    reset = [e for e in summary['events'] if e['event'] == 'map_reset']
    assert reset and 'map reset' in reset[0]['reason']
    assert summary['exploration']['map_resets'] and summary['exploration']['failures'] == 0
    assert summary['stats']['map_resets'] >= 1


class _Odom:
    def __init__(self, x, y):
        self.x, self.y = x, y


def _drift_stub(pose, obstacle_ahead=None):
    class S:
        pass
    s = S()
    s.events = []
    s.event = lambda name, **kw: s.events.append((name, kw))
    s._pose = lambda: pose
    s.robot = S()
    s.robot.can_actuate = True
    s.robot.telemetry = S()
    s.robot.telemetry.odom = S()
    s.robot.telemetry.odom.latest = lambda: (0.0, _Odom(pose[0], pose[1]))
    s._terrain_now = lambda now: type('T', (), {'obstacle_ahead_m': obstacle_ahead})()
    s.stats, s.record, s.nudge = {}, None, None
    return s


def test_drift_back_during_a_turn_triggers_a_forward_step():
    """Operator 2026-11-23: 'it backs up a bit, stop it or move it forward'."""
    s = _drift_stub((0.80, 0.0, 0.0))                                  # facing +x, started the turn at x 1.0
    R.RoamSession._drift_check(s, 0.0, {'x': 1.0, 'y': 0.0})
    assert s.nudge is not None and abs(s.nudge['length'] - 0.20) < 1e-6
    assert s.events[0][0] == 'drift_correction'
    s = _drift_stub((1.05, 0.0, 0.0))                                  # moved forward / barely: nothing
    R.RoamSession._drift_check(s, 0.0, {'x': 1.0, 'y': 0.0})
    assert s.nudge is None and not s.events
    s = _drift_stub((0.80, 0.0, 0.0), obstacle_ahead=0.35)            # something ahead: stay put
    R.RoamSession._drift_check(s, 0.0, {'x': 1.0, 'y': 0.0})
    assert s.nudge is None and s.events[0][0] == 'drift_not_corrected'
    s = _drift_stub((0.0, 0.0, 0.0))                                   # big slide: capped
    R.RoamSession._drift_check(s, 0.0, {'x': 1.0, 'y': 0.0})
    assert abs(s.nudge['length'] - R.NUDGE_MAX_M) < 1e-6


def test_forward_step_moves_the_simulated_robot_and_roaming_goes_on(tmp_path, monkeypatch, overlay):
    original = R.RoamSession._drift_check
    state = {'done': False}

    def force_once(self, now, spin):
        if not state['done'] and self.t_roam is not None and now - self.t_roam > 3:
            state['done'] = True
            odom = self.robot.telemetry.odom.latest()
            self.nudge = {'origin': (odom[1].x, odom[1].y), 't': now, 'length': 0.2, 'progress': (now, 0.0)}
            self.event('drift_correction', drifted_m=0.2, forward_m=0.2)
            return
        return original(self, now, spin)
    monkeypatch.setattr(R.RoamSession, '_drift_check', force_once)
    _, summary, robot, _ = roam(tmp_path, cfg=cfg(), stop_after=90)
    ev = summary['events']
    if not state['done']:
        pytest.skip('no turn happened in this simulated run')
    end = next(e for e in ev if e['event'] == 'drift_correction_end')
    assert end['how'] in ('done', 'obstacle ahead')
    vel = [s for s in robot.sent if s[1] == 'vel' and s[2] > 0 and s[3] == 0.0]
    assert vel and max(v[2] for v in vel) <= R.NUDGE_SPEED_M_S + 1e-9         # forward only, at nudge speed
    names = [e['event'] for e in ev]
    assert any(x in names[names.index('drift_correction_end') + 1:] for x in ('walk', 'spin', 'look_around'))


def test_look_around_given_up_after_two_short_turns():
    class S:
        pass
    s = S()
    s.events = []
    s.event = lambda name, **kw: s.events.append(name)
    s.stats = {'spins_completed': 0}
    s.look = [math.radians(-45)] * 4
    s.look_short = 0
    s._drift_check = lambda now, spin: None
    for k in range(2):
        s.goal = {'kind': 'spin', 'look': True, 'angle': math.radians(45), 'x': 0, 'y': 0}
        R.RoamSession._end_spin(s, float(k), 'incomplete', math.radians(10))
    assert s.look == [] and 'look_abandoned' in s.events
