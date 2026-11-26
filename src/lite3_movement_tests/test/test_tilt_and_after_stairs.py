"""Operator 2026-11-26 (18:14 run): keep going while tilted (step off it, never pause for it, no look-around on a
block) and keep exploring after the stairs (a failed crossing does not pause; the whole flight counts as crossed)."""
import inspect
import math

from lite3_movement_tests import roam as R

S = R.RoamSession


class Tel:
    def __init__(self, roll=0.0, pitch=0.0):
        self.tilt = (roll, pitch)

    def current_tilt(self):
        return self.tilt

    class odom:
        @staticmethod
        def latest():
            return (0.0, type('O', (), {'x': 0.0, 'y': 0.0})())


class Fake:
    _tilted = S._tilted
    _tilt_escape_tick = S._tilt_escape_tick
    _plan_look = S._plan_look
    _mark_flight_crossed = S._mark_flight_crossed

    def __init__(self, roll=0.0, pitch=0.0):
        self.robot = type('Rb', (), {'can_actuate': True, 'telemetry': Tel(roll, pitch)})()
        self.nudge, self.stairs, self.on_block, self.events, self.stats = None, None, None, [], {}
        self.last_pose = (0.0, 0.0, 0.0)

    def event(self, name, **kw):
        self.events.append((name, kw))

    def _pose(self):
        return self.last_pose


def test_tilted_and_still_steps_off_toward_the_lower_side_then_the_other_way():
    f = Fake(roll=math.radians(21.0), pitch=math.radians(-12.0))         # nose up: lower side behind
    pose = (1.0, 2.0, 0.0)
    assert f._tilt_escape_tick(10.0, pose) is False                     # just tilted: wait the hold time
    assert f._tilt_escape_tick(10.0 + R.TILT_ESCAPE_HOLD_S, pose) is True
    assert f.nudge['sign'] == -1 and f.nudge['tilt'] > 0 and f.nudge['stop_m'] == 0.0
    assert f.nudge['rear_m'] == R.TILT_ESCAPE_REAR_M and f.events[-1][0] == 'tilt_escape'
    f.nudge = None
    t = f._tilt_escape['next']
    assert f._tilt_escape_tick(t, pose) is True and f.nudge['sign'] == 1   # next attempt the other way
    for _ in range(R.TILT_ESCAPE_TRIES):
        f.nudge = None
        f._tilt_escape_tick(f._tilt_escape['next'], pose)
    f.nudge = None
    assert f._tilt_escape_tick(f._tilt_escape['next'] + 1, pose) is False     # tried here: the normal choice


def test_level_robot_never_steps_off():
    f = Fake(roll=math.radians(4.0))
    assert f._tilt_escape_tick(10.0, (0, 0, 0)) is False and f._tilt_escape_tick(20.0, (0, 0, 0)) is False


def test_the_step_off_ends_when_level_and_has_no_tilt_growth_stop():
    src = inspect.getsource(S._nudge_tick)
    assert "done = 'level'" in src and 'tilt growing' not in src        # no added safety stop (operator)


def test_no_look_around_on_a_block_or_while_tilted():
    f = Fake(roll=math.radians(15.0))
    f.look_check = True
    assert f._plan_look((0, 0, 0)) is False and f.events[-1] == ('look_skipped', {'reason': 'tilted'})
    g = Fake()
    g.on_block = {'t': 0}
    g.look_check = True
    assert g._plan_look((0, 0, 0)) is False and g.events[-1][1]['reason'] == 'on a block'


def test_the_block_rule_does_not_expire_while_still_tilted():
    src = inspect.getsource(S._on_block_filter)
    assert 'and not self._tilted()' in src


def test_a_failed_crossing_keeps_exploring_without_a_pause():
    src = inspect.getsource(S._stairs_tick)
    aborted = src.split('if action.aborted:')[-1][:300]
    assert "then='keep exploring'" in aborted and '_pause' not in aborted


def test_the_whole_flight_counts_as_crossed():
    f = Fake()
    f._pi_stairs_crossed = [(1.0, 0.0)]                    # where the stairs were seen
    f.last_pose = (2.6, 0.0, 0.0)                           # after the climb, facing on
    f._mark_flight_crossed()
    pts = f._pi_stairs_crossed
    far = max(p[0] for p in pts)
    assert far == pytest_approx(3.4) and all(abs(p[1]) < 1e-9 for p in pts)
    seen_from_top = (3.0, 0.5)                              # 06:24: the same flight seen again from the top
    assert min(math.dist(seen_from_top, p) for p in pts) <= R.PI_STAIRS_SAME_M


def pytest_approx(v):
    import pytest
    return pytest.approx(v, abs=0.01)
