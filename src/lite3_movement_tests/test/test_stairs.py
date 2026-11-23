"""Stair routine (stairs.py): pure state machine, no ROS, no robot."""
import math

import pytest

from lite3_movement_tests.stairs import HStepGaitStairs, RobotStairMode, StairConfig, StairRoutine


def drive(routine, profile, dt=0.1, max_s=60.0):
    """profile(travelled) -> pitch; integrates the commanded speed. Returns (actions, switches)."""
    t, x, switches, last = 0.0, 0.0, [], None
    while t < max_s:
        a = routine.step(t, profile(x), 0.0, x)
        if a.switch:
            switches.append(a.switch)
        last = a
        if a.done or a.aborted:
            break
        x += a.vx * dt
        t += dt
    return last, switches


def stairs_profile(start=0.3, length=0.9, pitch=0.17):
    return lambda x: pitch if start <= x < start + length else 0.0


@pytest.mark.parametrize('direction, sign', [('up', 1), ('down', -1)])
def test_crosses_three_steps_and_restores_the_gait(direction, sign):
    last, switches = drive(StairRoutine(direction), lambda x: sign * stairs_profile()(x))
    assert last.done and switches == ['in', 'out']


def test_aborts_on_attitude_limits_and_switches_back():
    r = StairRoutine('up')
    assert r.step(0.0, 0.0, 0.0, 0.0).vx > 0
    a = r.step(0.1, 0.0, 0.5, 0.0)                        # roll beyond max_roll
    assert a.aborted and a.switch == 'out' and 'roll' in a.reason
    last, _ = drive(StairRoutine('up'), lambda x: 0.6 if x > 0.3 else 0.0)
    assert last.aborted and 'pitch' in last.reason


def test_climb_timeout_retries_then_aborts():
    cfg = StairConfig(max_climb_s=3.0, max_retries=1)
    last, switches = drive(StairRoutine('up', cfg), lambda x: 0.17 if x > 0.3 else 0.0, max_s=30)
    assert last.aborted and '1 retries' in last.reason and switches[0] == 'in'


def test_no_incline_and_missing_imu_abort():
    last, _ = drive(StairRoutine('up'), lambda x: 0.0)
    assert last.aborted and 'no incline' in last.reason
    assert StairRoutine('up').step(0.0, None, 0.0, 0.0).aborted
    assert StairRoutine('down').step(0.0, math.nan, 0.0, 0.0).aborted


def test_config_rejects_undocumented_speeds_and_bad_thresholds():
    with pytest.raises(ValueError):
        StairConfig(climb_speed_m_s=1.5)                  # > 1.0 m/s (Comm Interface s1.2.12)
    with pytest.raises(ValueError):
        StairConfig(level_pitch_rad=0.1, incline_pitch_rad=0.05)
    with pytest.raises(ValueError):
        StairRoutine('sideways')


def test_documented_interface_uses_gaits_and_the_unknown_mode_refuses():
    sent = []

    class Robot:
        def send_gait(self, name):
            sent.append(name)
            return {'published': True}
    iface = HStepGaitStairs(Robot())
    iface.switch_in('up')
    iface.switch_out()
    assert sent == ['rug_hstep', 'flat_medium']
    with pytest.raises(NotImplementedError):
        RobotStairMode().switch_in('up')


def test_ai_mode_interface_enters_drives_by_axis_and_exits_to_navigation_mode():
    from lite3_movement_tests import ros_io as IO
    from lite3_movement_tests.stairs import AiModeStairs
    log = []

    class Robot:
        def send_velocity(self, vx, wz):
            log.append(('vel', vx))

        def send_simple_command(self, code, value=0):
            log.append(('simple', code))
            return {'published': True}

        def send_axis(self, code, value):
            log.append(('axis', code, value))

        def send_gait(self, name):
            log.append(('gait', name))
    iface = AiModeStairs(Robot())
    iface.drive(0.1)                                     # before AI state: navigation-mode speed command
    iface.switch_in('up')
    iface.drive(0.1)
    iface.drive(0.0)
    iface.switch_out()
    assert log == [('vel', 0.1), ('vel', 0.0), ('simple', IO.AI_ENTER), ('simple', IO.AI_GAIT_BASIC),
                   ('axis', IO.AXIS_FORWARD, 12000), ('axis', IO.AXIS_FORWARD, 0), ('axis', IO.AXIS_FORWARD, 0),
                   ('simple', IO.AI_EXIT), ('simple', IO.NAVIGATION_MODE), ('gait', 'flat_medium')]
    with pytest.raises(ValueError):
        AiModeStairs(Robot(), forward_value=5000)        # inside the documented dead zone (+-6553)


def test_routine_can_start_at_the_step_and_crosses_small_steps_without_tilt():
    r = StairRoutine('up', StairConfig(), start='switch_in')
    assert r.step(0.0, 0.0, 0.0, 0.0).switch == 'in'
    last, switches = drive(StairRoutine('up', start='switch_in'), lambda x: 0.0)   # 5 cm step: no body tilt
    assert last.done and switches == ['in', 'out']



def _run(routine, profile, dt=0.1, speed=0.2, limit=120.0):
    """Drive the routine through a pitch profile over distance; returns (actions, states visited)."""
    t, x, states, actions = 0.0, 0.0, [], []
    while t < limit:
        a = routine.step(t, profile(x), 0.0, x)
        actions.append(a)
        if not states or states[-1] != routine.state:
            states.append(routine.state)
        if a.done or a.aborted:
            break
        x += a.vx * dt
        t += dt
    return actions, states


def test_over_climbs_crosses_the_landing_and_descends_before_switching_out():
    """Competition staircase: 3 up (nose up), a landing, 3 down (nose down), then level ground."""
    from lite3_movement_tests.stairs import StairRoutine
    def pitch(x):
        if x < 0.3:
            return 0.0                                     # approach, flat
        if x < 1.2:
            return -0.17                                   # climbing (nose up)
        if x < 1.7:
            return 0.0                                     # top landing
        if x < 2.6:
            return 0.17                                    # descending (nose down)
        return 0.0
    r = StairRoutine('up', over=True)
    actions, states = _run(r, pitch)
    assert actions[-1].done and not actions[-1].aborted
    assert states == ['approach', 'switch_in', 'climb', 'landing', 'descend', 'switch_out', 'done']
    assert [a.switch for a in actions if a.switch] == ['in', 'out']          # stair mode held over the top
    assert r.direction == 'down'


def test_over_with_no_way_down_switches_out_on_the_landing():
    from lite3_movement_tests.stairs import StairRoutine
    r = StairRoutine('up', over=True)
    actions, states = _run(r, lambda x: -0.17 if 0.3 <= x < 1.2 else 0.0)
    assert actions[-1].done and states[-3:] == ['landing', 'switch_out', 'done']


def test_without_over_the_climb_ends_at_the_top_as_before():
    from lite3_movement_tests.stairs import StairRoutine
    _, states = _run(StairRoutine('up'), lambda x: -0.17 if 0.3 <= x < 1.2 else 0.0)
    assert 'landing' not in states and states[-1] == 'done'
