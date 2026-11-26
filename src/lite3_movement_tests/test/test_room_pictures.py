"""Operator 2026-11-26: a picture at the doorway before walking into a room (the capture inside still runs), and a
picture at every posture view. Labelling and reporting stay with the mission reporter (Pi YOLO frames)."""
import json
import math

from lite3_movement_tests import roam as R
from lite3_movement_tests.course_grid import CourseGrid

S = R.RoamSession


class Fake:
    _room_entry_start = S._room_entry_start
    _room_entry_snap = S._room_entry_snap
    _room_entry_tick = S._room_entry_tick
    _posture_picture = S._posture_picture
    _posture_view_done = S._posture_view_done
    _still_since = S._still_since          # 2026-11-26: the NUC measures "standing still" (no telemetry here:
    _still_enough = S._still_enough        # treated as still, as before)
    _still_fields = S._still_fields

    def __init__(self, frames=None):
        self.course = CourseGrid(0.0, 0.0, 0.0)
        self.requests, self.events, self.spins, self.walks, self.stats = [], [], [], [], {}
        self.goal, self.record, self.last_pose = None, None, (0.0, 0.0, 0.0)
        test = self

        class Robot:
            capture_acks = []
            pi_frames = frames

            def publish_capture_request(self, text):
                test.requests.append(json.loads(text))

            def send_velocity(self, vx, wz):
                pass
        self.robot = Robot()

    def event(self, name, **kw):
        self.events.append((name, kw))

    def _capture_uneven(self, now):
        return None

    def _pi_frames_since(self, t0):
        return None

    def _start_spin(self, now, pose, angle, reason=''):
        self.spins.append(round(math.degrees(angle)))
        self.goal = {'kind': 'spin'}

    def _start_walk(self, now, pose, goal):
        self.walks.append(goal)


ROOM = {'kind': 'walk', 'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'ends_at': 'room', 'cell': [1, 0]}


def test_facing_the_room_it_takes_the_picture_then_walks_in_on_the_answer():
    f = Fake()
    assert f._room_entry_start(10.0, (0.0, 0.0, 0.0), ROOM) is True
    req = f.requests[0]
    assert req['reason'] == 'room entry' and req['cell'] == [1, 0] and f.walks == []
    f._room_entry_tick(11.0, (0.0, 0.0, 0.0))
    assert f.walks == []                                         # holding for the answer
    f.robot.capture_acks = [(11.5, json.dumps({'id': req['id'], 'status': 'analysed', 'summary': 'a bedroom',
                                                'stored': True}))]
    f._room_entry_tick(12.0, (0.0, 0.0, 0.0))
    assert f.walks and f.walks[0]['doorway_done'] and f.walks[0]['ends_at'] == 'room'
    done = [kw for n, kw in f.events if n == 'room_entry_capture_done'][0]
    assert done['stored'] and 'bedroom' in done['result']


def test_without_an_answer_it_walks_in_after_the_limit():
    f = Fake()
    f._room_entry_start(10.0, (0.0, 0.0, 0.0), ROOM)
    f._room_entry_tick(10.0 + R.ROOM_ENTRY_MAX_S + 0.1, (0.0, 0.0, 0.0))
    assert f.walks and [kw for n, kw in f.events if n == 'room_entry_capture_done'][0]['result'] == 'no ack'


def test_not_facing_the_room_turns_first_then_takes_the_picture():
    f = Fake()
    assert f._room_entry_start(10.0, (0.0, 0.0, math.pi / 2), ROOM) is True
    assert f.spins == [-90] and f.requests == [] and f.goal['fixed_turn']
    f.goal = None
    f._room_entry_tick(14.0, (0.0, 0.0, 0.0))
    assert f.requests and f.requests[0]['reason'] == 'room entry'


def test_one_picture_per_room_and_none_from_inside():
    f = Fake()
    f._doorway_done = {(1, 0)}
    assert f._room_entry_start(10.0, (0.0, 0.0, 0.0), ROOM) is False
    g = Fake()
    assert g._room_entry_start(10.0, (1.1, 0.0, 0.0), ROOM) is False and not g.requests   # already in the room


def test_every_posture_view_takes_one_picture_once_settled():
    f = Fake()
    p = {'t': 5.0, 'round': 2}
    f._posture_view_done(5.5, p, 'left', 2.5)                       # not settled yet
    assert f.requests == []
    f._posture_view_done(6.2, p, 'left', 2.5)
    f._posture_view_done(6.6, p, 'left', 2.5)                       # same view: no second picture
    assert len(f.requests) == 1 and f.requests[0]['reason'] == 'posture view'
    assert f.requests[0]['view_id'] == 'left' and f.requests[0]['round'] == 2
    p['t'] = 9.0
    f._posture_view_done(10.2, p, 'right', 2.5)
    assert [r['view_id'] for r in f.requests] == ['left', 'right']


def test_room_scan_views_are_not_doubled():
    f = Fake()
    f._posture_view_done(7.0, {'t': 5.0, 'scan': {'id': 'scan-1'}}, 'left', 2.5)
    assert f.requests == []                                          # the room scan sends its own per view
