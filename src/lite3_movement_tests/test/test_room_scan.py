"""Operator 2026-11-26: room scan = posture look left then right (one round). At each settled side roam asks the
Pi for a fresh photo (capture_request with scan_id / view_id) and waits for its ack, retrying once; centred, it
waits for the scan result and takes one more look when Claude asks for it."""
from collections import deque
import json
import types

from lite3_movement_tests import roam as R


def run_scan(pi, until_s=90.0, dt=0.1):
    """pi(t, request_dict, acks, results) answers requests like the Pi would."""
    clock = {'t': 0.0}
    requests, events = [], []
    acks, results = deque(maxlen=20), deque(maxlen=20)

    def publish(text):
        req = json.loads(text)
        requests.append((clock['t'], req))

    robot = types.SimpleNamespace(
        now=lambda: clock['t'], send_velocity=lambda *a: None,
        send_simple_command=lambda code: {'published': True}, send_axis=lambda a, v: None,
        publish_capture_request=publish, capture_acks=acks, room_scan_msgs=results)
    s = R.RoamSession.__new__(R.RoamSession)
    s.robot, s.last_pose = robot, (1.0, 2.0, 0.0)
    s.runner = types.SimpleNamespace(own_axis_until=0.0)
    s.cfg = types.SimpleNamespace(arena_holds=False, gait=None)
    s.record = None
    s.event = lambda name, **kw: events.append((clock['t'], name, kw))
    s.log = lambda *_: None
    s.choose_after = 0.0
    s.posture_look = {'sides': ['left', 'right'], 'phase': 'enter', 't': None, 'max_rounds': 1,
                      'scan': {'id': 'scan-1', 'cell': [1, 2], 'pose': [1.0, 2.0, 0.0], 'views': {}, 'extra': 0}}
    while s.posture_look is not None and clock['t'] < until_s:
        s._posture_look_tick(clock['t'])
        pi(clock['t'], requests, acks, results)
        clock['t'] = round(clock['t'] + dt, 3)
    return s, requests, events


def good_pi(extra=None, drop=()):
    answered, sent = set(), {'result': False}

    def pi(t, requests, acks, results):
        for t_req, req in requests:
            if req['id'] in answered or t < t_req + 1.0:
                continue
            answered.add(req['id'])
            if req['id'] in drop:
                continue                                   # lost: roam must retry
            acks.append((t, json.dumps({'id': req['id'], 'status': 'captured', 'scan_id': req['scan_id'],
                                        'view_id': req['view_id'], 'capture_stamp_ns': int(t * 1e9), 'qr': []})))
            if req.get('final'):
                add = extra if not sent['result'] else None
                sent['result'] = True
                results.append((t + 2.0, json.dumps({'event': 'room_scan_result', 'scan_id': req['scan_id'],
                                                     'room': {'type': 'kitchen'}, 'findings': [], 'qr': [],
                                                     'additional_view': add or {'needed': False}})))
    return pi


def test_two_views_one_scan_then_leaves_posture():
    s, requests, events = run_scan(good_pi())
    reqs = [r for _, r in requests]
    assert [(r['view_id'], r['final'], r['scan_id'], r['reason']) for r in reqs] == [
        ('left', False, 'scan-1', 'room'), ('right', True, 'scan-1', 'room')]
    assert reqs[0]['cell'] == [1, 2] and reqs[0]['pose'] == [1.0, 2.0, 0.0]
    assert {v: st['status'] for v, st in s._last_scan['views'].items()} == {'left': 'captured', 'right': 'captured'}
    names = [e[1] for e in events]
    assert names.index('room_scan_view') < names.index('room_scan_request', names.index('room_scan_view'))
    assert 'room_scan_result' in names and names[-1] == 'posture_look_end'
    assert s._last_scan['result']['room']['type'] == 'kitchen'


def test_lost_ack_is_retried_once_with_a_new_id():
    s, requests, events = run_scan(good_pi(drop={'scan-1-left-1'}))
    ids = [r['id'] for _, r in requests]
    assert ids == ['scan-1-left-1', 'scan-1-left-2', 'scan-1-right-1']
    t1, t2 = requests[0][0], requests[1][0]
    assert abs((t2 - t1) - R.ROOM_SCAN_VIEW_WAIT_S) < 0.3
    assert s._last_scan['views']['left']['status'] == 'captured'


def test_view_given_up_after_two_attempts_and_the_look_moves_on():
    s, requests, events = run_scan(good_pi(drop={'scan-1-left-1', 'scan-1-left-2'}))
    assert s._last_scan['views']['left']['status'] == 'timeout'
    assert s._last_scan['views']['right']['status'] == 'captured'
    assert any(e[1] == 'room_scan_view_failed' for e in events)


def test_claude_asks_for_one_more_qr_view():
    extra = {'needed': True, 'view_id': 'right', 'reason': 'QR code too small to read'}
    s, requests, events = run_scan(good_pi(extra=extra))
    reqs = [r for _, r in requests]
    assert [r['view_id'] for r in reqs] == ['left', 'right', 'extra-right']
    assert reqs[-1]['qr_only'] is True and reqs[-1]['final'] is True and reqs[-1]['scan_id'] == 'scan-1'
    assert sum(1 for e in events if e[1] == 'room_scan_extra_view') == 1     # at most one extra look


def test_silent_pi_never_strands_the_dog():
    s, requests, events = run_scan(lambda *a: None)
    assert s.posture_look is None                                            # the look ended by itself
    assert all(st['status'] == 'timeout' for st in s._last_scan['views'].values())
    assert events[-1][1] == 'posture_look_end' and events[-1][0] < 2 * 2 * R.ROOM_SCAN_VIEW_WAIT_S + 15
