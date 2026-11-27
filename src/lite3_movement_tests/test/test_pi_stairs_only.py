"""--stairs-pi-only (operator 2026-11-26): the Pi's stairs YOLO alone starts the stair handover into AI mode.
No RGB-D depth, VLM or terrain confirmation; rollers and curtains never trigger it. Full roam loop on the simulator."""
import json

import pytest

from lite3_movement_tests import ros_io as IO
from lite3_movement_tests import roam as R
from test_competition_scenarios import cfg, first, names, staircase
from test_m10 import roam


def pi_stair_boxes(monkeypatch, after_s, mid_x=320.0, confidence=0.90, name='stairs', latency_s=1.5, period_s=1.7,
                   until_s=1e9):
    """The Pi's obstacle YOLO reporting one box every period_s from after_s on, arriving latency_s after capture."""
    original = R.RoamSession._take_scene
    state = {'next': None}

    def take(self, now, pose):
        if self.t_roam is not None and after_s <= now - self.t_roam <= until_s:
            if state['next'] is None or now >= state['next']:
                state['next'] = now + period_s
                wall = 1_000_000.0 + now
                msg = {'frame': 1, 'capture_stamp_ns': int((wall - latency_s) * 1e9),
                       'frame_id': 'camera_color_optical_frame', 'objects': [],
                       'stair_curtain': {'obstacle': 'unknown', 'reason': 'timestamp_mismatch'},
                       'stair_candidates': [{'name': name, 'class': 0, 'confidence': confidence,
                                             'box': {'x1': mid_x - 60, 'y1': 200.0, 'x2': mid_x + 60, 'y2': 330.0}}]}
                self.robot.pi_stair_objects = (now, wall, json.dumps(msg))
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)


def pi_only_cfg():
    c = cfg()
    c.stairs_pi_only = True
    return c


def ai_commands(robot):
    return [(s[1], s[2]) for s in robot.sent if s[1] == 'simple' and s[2] in (IO.AI_ENTER, IO.AI_EXIT)]


def test_stairs_ahead_hand_over_to_ai_mode_with_no_other_confirmation(tmp_path, monkeypatch):
    pi_stair_boxes(monkeypatch, 6.0, until_s=20.0)      # frames from mid-turn are ignored: keep reporting until still
    staircase(monkeypatch)
    c = pi_only_cfg()
    c.abort_clearance_m = 0.0          # as physical runs (--abort-clearance 0): the simulator has no ultrasound model
    _, summary, robot, _ = roam(tmp_path, stop_after=60, cfg=c)
    n = names(summary)
    seen, handover = first(summary, 'stairs_pi_seen'), first(summary, 'stairs_pi_handover')
    start = first(summary, 'stairs_start')
    assert seen['bearing_deg'] == 0.0 and 0.3 <= seen['range_m'] <= 1.5          # box bottom on the floor ahead
    assert handover['t_s'] - seen['t_s'] < 3.0                                     # at most a short facing turn
    assert start['mode'] == 'ai' and start['trigger'] == 'pi stairs yolo'
    assert n.index('stairs_pi_seen') < n.index('stairs_pi_handover') < n.index('stairs_start')
    # nothing waited for depth, terrain, the stop-and-confirm or a walk back
    for skipped in ('pi_obstacle_stop', 'stairs_pi_depth_confirmed', 'stairs_confirmation_wait',
                    'stairs_return_started', 'stairs_deferred'):
        assert skipped not in n
    assert (('simple', IO.AI_ENTER)) in ai_commands(robot) and 'stairs_done' in n


def test_stairs_off_to_the_side_turn_to_face_them_first(tmp_path, monkeypatch):
    # Operator 2026-11-26 switched the facing turn off (PI_STAIRS_ALIGN = False); this checks it still works when on.
    monkeypatch.setattr(R, 'PI_STAIRS_ALIGN', True)
    pi_stair_boxes(monkeypatch, 6.0, mid_x=80.0)                         # left third of the image: about +27 deg
    staircase(monkeypatch)
    _, summary, _, _ = roam(tmp_path, stop_after=60, cfg=pi_only_cfg())
    seen = first(summary, 'stairs_pi_seen')
    assert 20 <= seen['bearing_deg'] <= 35
    n = names(summary)
    turn = next(e for e in summary['events'] if e['event'] == 'spin' and e['t_s'] >= seen['t_s'])
    assert turn['degrees'] > 0                                           # towards the stairs (left)
    assert first(summary, 'stairs_pi_handover')['turned'] is True
    assert n.index('stairs_pi_seen') < n.index('stairs_pi_handover') < n.index('stairs_start')


def test_rollers_curtains_and_weak_boxes_never_start_ai_mode(tmp_path, monkeypatch):
    for case, kw in (('roller', dict(name='roller')), ('curtain', dict(name='curtain')),
                     ('weak', dict(confidence=R.PI_STAIRS_MIN_CONFIDENCE - 0.001))):
        with monkeypatch.context() as m:
            pi_stair_boxes(m, 6.0, **kw)
            (tmp_path / case).mkdir()
            _, summary, robot, _ = roam(tmp_path / case, stop_after=40, cfg=pi_only_cfg())
        n = names(summary)
        assert 'stairs_pi_handover' not in n and 'stairs_start' not in n, case
        assert ('simple', IO.AI_ENTER) not in ai_commands(robot), case


def test_off_by_default(tmp_path, monkeypatch):
    pi_stair_boxes(monkeypatch, 6.0)
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=cfg())
    assert 'stairs_pi_handover' not in names(summary)


def test_cli_flag():
    assert R.parse(['run', '--speed-limit', '0.5', '--stairs-pi-only']).stairs_pi_only is True
    assert R.parse(['run', '--speed-limit', '0.5']).stairs_pi_only is False


def test_stairs_already_handed_over_are_not_crossed_again():
    """Same flight seen again (within 1.2 m of where it was handed over): ignored; a flight elsewhere is new."""
    import types
    events = []
    session = R.RoamSession.__new__(R.RoamSession)
    session.cfg = types.SimpleNamespace(pi_advisory=True)
    session.event = lambda name, **kw: events.append((name, kw))
    session._pi_stair_poses = [(100.0, (0.0, 0.0, 0.0))]

    def sighting(now, mid_x=320.0, y2=330.0):
        wall = 1_000_000.0 + now
        msg = {'capture_stamp_ns': int((wall - 0.5) * 1e9), 'stair_candidates': [
            {'name': 'stairs', 'confidence': 0.9, 'box': {'x1': mid_x - 40, 'y1': 200.0, 'x2': mid_x + 40, 'y2': y2}}]}
        session.robot = types.SimpleNamespace(pi_stair_objects=(now, wall, json.dumps(msg)))
        return session._pi_new_stairs(now)

    first_seen = sighting(100.5)
    assert first_seen is not None and 0.3 < first_seen[2] < 1.0
    session._pi_stairs_crossed = [first_seen[3]]
    assert sighting(101.0) is None and events[-1][0] == 'stairs_pi_already_crossed'
    session._pi_stair_poses = [(102.0, (3.0, 0.0, 0.0))]      # seen from 3 m further on: another flight
    assert sighting(102.5) is not None


def test_yolo_sighting_switches_to_ai_mode_at_once_without_a_tilt_approach():
    """Operator 2026-11-26: AI mode when YOLO sees the stairs, not when the body tilts on the first step."""
    import inspect
    from lite3_movement_tests import roam as R
    src = inspect.getsource(R.RoamSession._pi_stairs_handover)
    assert "_begin_stairs('up', 'switch_in', 'pi stairs yolo'" in src


def test_a_sighting_from_before_a_turn_is_ignored():
    """Run 26 Nov 05:50: YOLO saw the stairs mid-turn; by the handover the robot had turned 64 deg past them."""
    import math
    from types import SimpleNamespace
    events = []

    def session(yaw_now_deg):
        s = SimpleNamespace(event=lambda name, **kw: events.append((name, kw)), _pi_stairs_crossed=[])
        s._pi_stairs_reading = lambda now: (10.0, 0.0, 0.6)                     # captured t=10, dead ahead, 0.6 m
        s._pi_stairs_place = lambda pose, bearing, rng: (pose[0] + rng, pose[1])
        s._pi_stair_poses = [(10.0, (0.0, 0.0, 0.0)), (11.5, (0.0, 0.0, math.radians(yaw_now_deg)))]
        return s
    turning = session(60.0)
    assert R.RoamSession._pi_new_stairs(turning, 11.5) is None
    assert events[-1][0] == 'stairs_pi_stop_to_look' and events[-1][1]['rotated_deg'] == 60.0
    assert turning._pi_stairs_settle_until == pytest.approx(11.5 + R.PI_STAIRS_SETTLE_S)   # stop, look again
    still = session(3.0)
    assert R.RoamSession._pi_new_stairs(still, 11.5) is not None                # 3 deg: within 8, handed over
