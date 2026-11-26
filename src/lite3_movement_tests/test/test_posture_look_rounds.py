"""Operator 2026-11-26: at a wall, the posture look (pose-mode body twist) runs 3 times with a 3 s pause between
rounds, body centred, then pose mode is left once."""
import types

from lite3_movement_tests import roam as R
from lite3_movement_tests.ros_io import AXIS_YAW, MOVE_MODE, NAVIGATION_MODE, POSE_MODE


def run_look(sides, until_s=40.0, dt=0.1):
    clock = {'t': 0.0}
    axis, simple = [], []
    robot = types.SimpleNamespace(
        now=lambda: clock['t'], send_velocity=lambda *a: None,
        send_simple_command=lambda code: simple.append((clock['t'], code)) or {'published': True},
        send_axis=lambda a, v: axis.append((clock['t'], a, v)))
    events = []
    s = R.RoamSession.__new__(R.RoamSession)
    s.robot, s.last_pose = robot, (0.0, 0.0, 0.0)
    s.runner = types.SimpleNamespace(own_axis_until=0.0)
    s.cfg = types.SimpleNamespace(arena_holds=False, gait=None)
    s.event = lambda name, **kw: events.append((clock['t'], name, kw))
    s.log = lambda *_: None
    s.posture_look = {'sides': sides, 'phase': 'enter', 't': None}
    while s.posture_look is not None and clock['t'] < until_s:
        s._posture_look_tick(clock['t'])
        clock['t'] = round(clock['t'] + dt, 3)
    return events, axis, simple


def test_three_rounds_with_three_second_pauses():
    assert (R.POSTURE_LOOK_ROUNDS, R.POSTURE_LOOK_GAP_S) == (3, 3.0)
    events, axis, simple = run_look(['left', 'right'])
    names = [e[1] for e in events]
    rounds = [e for e in events if e[1] == 'posture_look_round_done']
    assert [e[2]['round'] for e in rounds] == [1, 2]
    assert names[0] == 'posture_look_start' and names[-1] == 'posture_look_end'
    assert [c for _, c in simple].count(POSE_MODE) == 1                       # pose mode entered once ...
    assert [c for _, c in simple][-2:] == [MOVE_MODE, NAVIGATION_MODE]        # ... and left once, at the end
    # each pause holds the body centred for 3 s, then the next round twists left again
    for _, _, kw in rounds:
        assert kw['pause_s'] == 3.0
    for t_done, _, _ in rounds:
        during = [v for t, a, v in axis if a == AXIS_YAW and t_done < t < t_done + 2.9]
        assert during and all(v == 0 for v in during)
        after = [v for t, a, v in axis if a == AXIS_YAW and t_done + 3.05 <= t <= t_done + 3.3]
        assert after and after[0] == -R.POSTURE_LOOK_YAW_AXIS                 # left again
    lefts = [t for t, a, v in axis if a == AXIS_YAW and v == -R.POSTURE_LOOK_YAW_AXIS]
    starts = [t for i, t in enumerate(lefts) if i == 0 or t - lefts[i - 1] > 0.15]
    assert len(starts) == 3                                                   # three separate left twists
    total = events[-1][0]
    per_round = 2 * R.POSTURE_LOOK_HOLD_S + R.POSTURE_LOOK_CENTRE_S
    assert abs(total - (3 * per_round + 2 * R.POSTURE_LOOK_GAP_S)) < 0.5


def test_one_side_also_three_rounds():
    events, _, _ = run_look(['right'])
    assert [e[2]['round'] for e in events if e[1] == 'posture_look_round_done'] == [1, 2]
    assert events[-1][1] == 'posture_look_end'


def run_look_with_pi(sides, frame_period_s, latency_s=1.5, until_s=120.0, dt=0.1, silent=False):
    """Like run_look, with a Pi stream: one detections frame every frame_period_s, arriving latency_s after capture."""
    from collections import deque
    clock = {'t': 0.0}
    robot = types.SimpleNamespace(
        now=lambda: clock['t'], send_velocity=lambda *a: None,
        send_simple_command=lambda code: {'published': True}, send_axis=lambda a, v: None,
        pi_frames=deque(maxlen=60))
    events = []
    s = R.RoamSession.__new__(R.RoamSession)
    s.robot, s.last_pose = robot, (0.0, 0.0, 0.0)
    s.runner = types.SimpleNamespace(own_axis_until=0.0)
    s.cfg = types.SimpleNamespace(arena_holds=False, gait=None)
    s.event = lambda name, **kw: events.append((clock['t'], name, kw))
    s.log = lambda *_: None
    s.posture_look = {'sides': sides, 'phase': 'enter', 't': None}
    next_capture = 0.0
    while s.posture_look is not None and clock['t'] < until_s:
        if not silent and clock['t'] >= next_capture + latency_s:   # the frame captured at next_capture arrives now
            wall = 1_000_000.0 + clock['t']
            robot.pi_frames.append((clock['t'], wall, int((wall - latency_s) * 1e9)))
            next_capture += frame_period_s
        s._posture_look_tick(clock['t'])
        clock['t'] = round(clock['t'] + dt, 3)
    return events


def test_each_view_waits_for_two_pi_frames_captured_after_it_settled():
    events = run_look_with_pi(['left', 'right'], frame_period_s=1.6)      # ~0.6 frames/s, 1.5 s behind
    views = [kw for _, name, kw in events if name == 'posture_view_held']
    assert [v['view'] for v in views] == ['left', 'right', 'centre'] * 3
    assert all(v['pi_frames'] >= 2 and not v['timed_out'] for v in views)
    # two frames at 1.6 s apart, after 1 s settle, arriving 1.5 s late: far longer than the old 2.5 s hold
    assert all(v['held_s'] >= 4.0 for v in views if v['view'] != 'centre')
    assert all(v['held_s'] <= R.POSTURE_LOOK_MAX_S for v in views)


def test_a_silent_pi_never_holds_a_view_beyond_the_maximum():
    events = run_look_with_pi(['left'], frame_period_s=1.6, silent=True)
    views = [kw for _, name, kw in events if name == 'posture_view_held']
    assert views and all(v['timed_out'] and v['held_s'] <= R.POSTURE_LOOK_MAX_S + 0.15 for v in views)
    assert events[-1][1] == 'posture_look_end'


def test_after_a_backtrack_it_faces_where_it_was_stuck_and_engages_the_pose():
    import math
    events, spins = [], []
    s = R.RoamSession.__new__(R.RoamSession)
    s.robot = types.SimpleNamespace(can_actuate=True, send_axis=lambda *a: None)
    s.goal, s.posture_look = None, None
    s.event = lambda name, **kw: events.append((name, kw))

    def start_spin(now, pose, angle, reason=''):
        spins.append(math.degrees(angle))
        s.goal = {'kind': 'spin'}
    s._start_spin = start_spin
    s.backtrack_scan = {'at': (0.0, 2.0), 'phase': 'face'}                 # stuck spot straight to the left
    assert s._backtrack_scan_tick(10.0, (0.0, 0.0, 0.0)) is True
    assert len(spins) == 1 and abs(spins[0] - 90.0) < 1e-6                  # turn to face it first
    assert s._backtrack_scan_tick(11.0, (0.0, 0.0, math.pi / 2)) is False   # turning: nothing else
    s.goal = None
    assert s._backtrack_scan_tick(12.0, (0.0, 0.0, math.pi / 2)) is True
    assert s.posture_look == {'sides': ['left', 'right'], 'phase': 'enter', 't': None}
    assert events[-1][0] == 'backtrack_scan' and events[-1][1]['posture'] is True
    assert s.backtrack_scan is None and s._backtrack_scan_tick(13.0, (0.0, 0.0, 0.0)) is False
