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
    last, _ = drive(StairRoutine('up'), lambda x: 0.7 if x > 0.3 else 0.0)
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
    assert log == [('vel', 0.1), ('vel', 0.0), ('simple', IO.MANUAL_MODE), ('simple', IO.AI_ENTER),
                   ('simple', IO.AI_GAIT_BASIC),
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


def _switch_times(routine, profile, dt=0.1, max_s=60.0):
    """drive(), but records when each switch happened."""
    t, x, times, last = 0.0, 0.0, [], None
    while t < max_s:
        a = routine.step(t, profile(x), 0.0, x)
        if a.switch:
            times.append((a.switch, round(t, 2)))
        last = a
        if a.done or a.aborted:
            break
        x += a.vx * dt
        t += dt
    return last, times


def test_min_hold_keeps_the_stair_mode_on_after_switch_in():
    """Run 26 Nov 03:05: in at the first step, the robot was level again by the end of the settle, so a level,
    'already crossed' exit came 0.4 s into the climb. With min_hold_s the exit waits, walking on."""
    cfg = StairConfig(switch_settle_s=3.0, flat_cross_m=0.1, min_hold_s=10.0)
    r = StairRoutine('up', cfg, start='switch_in')
    last, times = _switch_times(r, lambda x: 0.0)
    assert last.done and [s for s, _ in times] == ['in', 'out']
    assert times[1][1] - times[0][1] >= 10.0 and r.hold_deferred > 0
    # without the hold the same profile switches out as soon as the settle and the distance allow
    last, times = _switch_times(StairRoutine('up', StairConfig(switch_settle_s=3.0, flat_cross_m=0.1),
                                             start='switch_in'), lambda x: 0.0)
    assert times[1][1] - times[0][1] < 4.0


def test_min_hold_counts_from_switch_in_and_never_delays_an_abort():
    cfg = StairConfig(switch_settle_s=1.0, min_hold_s=10.0)
    r = StairRoutine('up', cfg)
    last, times = _switch_times(r, lambda x: 0.7 if x > 0.5 else (0.1 if x > 0.3 else 0.0))
    assert last.aborted and 'pitch' in last.reason      # attitude limit: out at once, inside the hold
    assert times[-1][0] == 'out' and times[-1][1] - times[0][1] < 10.0
    # a long crossing already past the hold switches out at the bottom as before
    last, switches = drive(StairRoutine('up', StairConfig(min_hold_s=2.0), over=True),
                           lambda x: 0.17 if 0.3 <= x < 1.2 else (-0.17 if 1.6 <= x < 2.5 else 0.0))
    assert last.done and switches == ['in', 'out']


def test_min_hold_rejects_negative():
    with pytest.raises(ValueError):
        StairConfig(min_hold_s=-1.0)


def test_ai_mode_turns_the_robot_stops_off_before_ai_state_and_back_on_after():
    """Run 26 Nov 04:03: 43 s in AI state at the first step, forward sent every tick, no movement."""
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
            log.append(('axis', value))

        def send_gait(self, name):
            log.append(('gait', name))

        def send_ai_option(self, name):
            log.append(('option', name))
            if name == 'obstacle_avoidance':
                raise OSError('perception host unreachable')
            return {'published': True}
    iface = AiModeStairs(Robot())
    iface.switch_in('up')
    assert log[:5] == [('vel', 0.0), ('option', 'all_off'), ('option', 'avoidance_off'),
                       ('simple', IO.MANUAL_MODE), ('simple', IO.AI_ENTER)]
    assert iface.options_evidence['robot_stops'] == 'off'
    log.clear()
    iface.switch_out()
    assert log == [('axis', 0), ('simple', IO.AI_EXIT), ('simple', IO.NAVIGATION_MODE), ('option', 'auto_stop'),
                   ('option', 'obstacle_avoidance'), ('gait', 'flat_medium')]
    ev = iface.options_evidence
    assert ev['robot_stops'] == 'on' and ev['obstacle_avoidance']['published'] is False   # reported, not raised
    log.clear()
    iface.switch_out()                                    # a second exit does not switch them again
    assert ('option', 'auto_stop') not in log
    log.clear()
    off = AiModeStairs(Robot(), robot_stops_off=False)
    off.switch_in('up')
    off.switch_out()
    assert not [e for e in log if e[0] == 'option']


def test_perception_options_go_to_the_perception_host(monkeypatch):
    from lite3_movement_tests import ros_io as IO
    sent = []
    monkeypatch.setattr(IO, 'send_perception_udp', lambda code, value, host: sent.append((code, value)) or True)
    robot = IO.RosRobot.__new__(IO.RosRobot) if hasattr(IO, 'RosRobot') else None
    if robot is None:
        pytest.skip('no RosRobot class')
    monkeypatch.setattr(type(robot), 'send_simple_command', lambda self, code, value=0: sent.append(('motion', value)))
    robot.send_ai_option('avoidance_off')
    robot.send_ai_option('obstacle_avoidance')
    robot.send_ai_option('all_off')
    assert sent[:2] == [(IO.AI_OPTIONS_CODE, 0x00), (IO.AI_OPTIONS_CODE, 0x40)] and sent[2] == ('motion', 0x00)


def test_ai_mode_switches_to_manual_mode_before_ai_state_and_back_to_navigation_after():
    """Comm Interface s1.2.3/s1.2.7: axis commands are controller commands, taken only in manual mode."""
    from lite3_movement_tests import ros_io as IO
    from lite3_movement_tests.stairs import AiModeStairs
    codes = []

    class Robot:
        def send_velocity(self, vx, wz):
            pass

        def send_simple_command(self, code, value=0):
            codes.append(code)
            return {'published': True}

        def send_axis(self, code, value):
            pass

        def send_gait(self, name):
            pass
    iface = AiModeStairs(Robot())
    iface.switch_in('up')
    iface.switch_out()
    assert IO.MANUAL_MODE == 0x21010C02
    assert codes == [IO.MANUAL_MODE, IO.AI_ENTER, IO.AI_GAIT_BASIC, IO.AI_EXIT, IO.NAVIGATION_MODE]


def test_one_pitch_limit_of_35_degrees_everywhere():
    """Operator 2026-11-26: the stair, roam, curtain and sequence tilt limits are all 35 degrees."""
    from lite3_movement_tests import config as C, roam as R
    assert C.MAX_PITCH_RAD == pytest.approx(math.radians(35.0))
    assert StairConfig().max_pitch_rad == R.ROAM_MAX_TILT_RAD == R.CURTAIN_MAX_TILT_RAD == \
        C.SafetyLimits().max_tilt_rad == C.MAX_PITCH_RAD
    assert R.WATCHDOG_MAX_TILT_RAD == pytest.approx(C.MAX_PITCH_RAD + 0.05)
    r = StairRoutine('up')
    r.step(0.0, 0.0, 0.0, 0.0)
    assert not r.step(0.1, math.radians(34.0), 0.0, 0.0).aborted
    assert r.step(0.2, math.radians(36.0), 0.0, 0.0).aborted


def _walk(cfg, ahead, pitch=lambda x: 0.17 if 0.1 <= x < 0.5 else 0.0):
    """AI-mode walk from the step (start='switch_in'); ahead(x) -> ultrasound m, pitch(x) -> body pitch (default: the
    body tilts onto the stairs 0.1-0.5 m in). Returns (routine, last action, x when switched out)."""
    r = StairRoutine('up', cfg, start='switch_in')
    t, x, out_at, last = 0.0, 0.0, None, None
    while t < 120:
        a = r.step(t, pitch(x), 0.0, x, ahead(x))
        if a.switch == 'out':
            out_at = x
        last = a
        if a.done or a.aborted:
            break
        x += a.vx * 0.1
        t += 0.1
    return r, last, out_at


def test_ai_walk_ends_after_1_5_m_when_nothing_is_ahead():
    cfg = StairConfig(switch_settle_s=3.0, obstacle_stop_m=0.10, walk_m=1.5, max_climb_s=20.0)
    r, last, out_at = _walk(cfg, lambda x: 2.0)
    assert last.done and r.walk_end == 'distance' and out_at == pytest.approx(1.5, abs=0.05)


def test_ai_walk_stops_10_cm_before_an_obstacle_inside_the_1_5_m():
    cfg = StairConfig(switch_settle_s=3.0, obstacle_stop_m=0.10, walk_m=1.5, max_climb_s=20.0)
    r, last, out_at = _walk(cfg, lambda x: max(0.0, 0.9 - x))           # wall 0.9 m ahead of the start
    assert last.done and r.walk_end == 'obstacle' and r.obstacle_stop <= 0.10
    assert out_at == pytest.approx(0.8, abs=0.05)                        # stopped 10 cm short, not at 1.5 m


def test_ai_walk_without_a_reading_walks_on_to_1_5_m():
    """A missing front-sensor reading never stops the walk (operator 2026-11-26)."""
    cfg = StairConfig(switch_settle_s=1.0, obstacle_stop_m=0.10, walk_m=1.5, max_climb_s=20.0)
    r, last, out_at = _walk(cfg, lambda x: None)
    assert last.done and r.walk_end == 'distance' and out_at == pytest.approx(1.5, abs=0.05)


def test_ai_walk_ignores_the_front_sensor_until_it_has_climbed():
    """Run 06:02: 16 cm from the riser it stood still for 40 s. The 10 cm stop counts only once on the stairs."""
    cfg = StairConfig(switch_settle_s=1.0, obstacle_stop_m=0.10, walk_m=1.5, max_climb_s=20.0)
    r, last, out_at = _walk(cfg, lambda x: 0.05, pitch=lambda x: 0.17 if x >= 0.25 else 0.0)   # riser reads 5 cm
    assert last.done and r.walk_end == 'obstacle' and out_at == pytest.approx(0.25, abs=0.05)   # walked on, climbed
    never = _walk(cfg, lambda x: 0.05, pitch=lambda x: 0.0)                                      # never tilts
    assert never[0].walk_end == 'distance'


def test_ai_mode_walks_faster_by_default():
    """Operator 2026-11-26: AI mode faster - forward joystick value 22000 (was 18000, 12000)."""
    from lite3_movement_tests import roam as R
    assert R.AI_AXIS_FORWARD == 22000 and R.RoamConfig().ai_axis_forward == 22000
    assert R.build_parser().parse_args(['run']).ai_axis_forward == 22000 if hasattr(R, 'build_parser') else True
