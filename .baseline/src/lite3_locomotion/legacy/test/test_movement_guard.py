"""Deterministic guard tests; no ROS, sockets, or movement interface."""
import math

import pytest

from lite3_locomotion.command_policy import Velocity, ZERO
from lite3_legacy.movement_guard import MovementGuard, scan_is_clear
from lite3_legacy.guarded_adapter import parse_options


@pytest.fixture
def gate():
    now = [0.0]
    return MovementGuard(clock=lambda: now[0]), now


def test_disarmed_ignores_commands_and_unhealthy_arm(gate):
    guard, _ = gate
    assert not guard.accept(Velocity(linear_x=0.1), True)
    assert not guard.arm(False)
    assert guard.output(True) == ZERO
    assert not guard.ever_armed


def test_limits_and_disabled_lateral_axis(gate):
    guard, _ = gate
    assert guard.arm(True)
    assert guard.output(True) == ZERO
    assert guard.accept(Velocity(linear_x=3, linear_y=2, angular_z=-3), True)
    assert guard.output(True) == Velocity(linear_x=0.1, angular_z=-0.2)


@pytest.mark.parametrize('command', [Velocity(linear_x=math.nan), Velocity(angular_z=math.inf),
                                    Velocity(linear_z=0.01), Velocity(angular_x=1), None])
def test_invalid_command_latches_stop(gate, command):
    guard, _ = gate
    guard.arm(True)
    guard.accept(Velocity(linear_x=0.04), True)
    assert not guard.accept(command, True)
    assert guard.output(True) == ZERO
    assert not guard.armed
    assert guard.reason == 'invalid_command'
    assert not guard.accept(Velocity(linear_x=0.04), True)


def test_timeout_latched_and_rearm_requires_new_command(gate):
    guard, now = gate
    guard.arm(True)
    guard.accept(Velocity(linear_x=0.04), True)
    now[0] = 0.399
    assert guard.output(True).linear_x == 0.04
    now[0] = 0.4
    assert guard.output(True) == ZERO
    assert guard.reason == 'command_timeout'
    assert not guard.accept(Velocity(linear_x=0.04), True)
    assert guard.arm(True)
    assert guard.output(True) == ZERO
    assert guard.accept(Velocity(linear_x=0.03), True)
    assert guard.output(True).linear_x == 0.03


def test_operator_stop_clears_command(gate):
    guard, _ = gate
    guard.arm(True)
    guard.accept(Velocity(linear_x=0.04), True)
    guard.stop()
    assert guard.output(True) == ZERO
    assert guard.reason == 'operator_stop'
    guard.arm(True)
    assert guard.output(True) == ZERO


@pytest.mark.parametrize('where', ['accept', 'output'])
def test_sensor_loss_latches_stop(gate, where):
    guard, _ = gate
    guard.arm(True)
    guard.accept(Velocity(linear_x=0.04), True)
    if where == 'accept':
        assert not guard.accept(Velocity(), False)
    else:
        assert guard.output(False) == ZERO
    assert guard.output(True) == ZERO
    assert not guard.armed
    assert guard.reason == 'sensor_tf_or_link_unhealthy'


def test_first_command_deadline(gate):
    guard, now = gate
    guard.arm(True)
    now[0] = 1.999
    assert guard.output(True) == ZERO
    assert guard.armed
    now[0] = 2.0
    assert guard.output(True) == ZERO
    assert guard.reason == 'first_command_timeout'


def test_repeated_arm_does_not_extend_deadline(gate):
    guard, now = gate
    guard.arm(True)
    now[0] = 59.99
    assert guard.arm(True)
    guard.accept(Velocity(linear_x=0.04), True)
    assert guard.output(True).linear_x == 0.04
    now[0] = 60
    assert guard.output(True) == ZERO
    assert guard.reason == 'session_deadline'


def test_clock_regression_stops(gate):
    guard, now = gate
    guard.arm(True)
    guard.accept(Velocity(linear_x=0.04), True)
    now[0] = -1
    assert guard.output(True) == ZERO
    assert not guard.armed


@pytest.mark.parametrize('duration', [0, -1, 301, math.inf, math.nan])
def test_bad_session_duration(duration):
    with pytest.raises(ValueError):
        MovementGuard(session_seconds=duration)


@pytest.mark.parametrize('ranges, expected', [
    ([2.0] * 360, True), ([0.4] * 360, True), ([0.399] + [2.] * 359, False),
    ([math.inf] * 360, False), ([math.nan] * 360, False),
    ([2.] * 3 + [math.nan] * 357, False), ([2.] * 300 + [math.inf] * 60, True),
    ([2.] * 300 + [math.nan] * 60, True), ([2.] * 250 + [math.nan] * 110, False),
    ([2.] * 7, False), ([], False),
])
def test_scan_quality_and_clearance(ranges, expected):
    assert scan_is_clear(ranges, 0.1, 10.) is expected


@pytest.mark.parametrize('low, high', [(1., 1.), (-1., 10.), (0.1, math.inf)])
def test_invalid_scan_bounds(low, high):
    assert not scan_is_clear([2.] * 360, low, high)


@pytest.mark.parametrize('args', [['--ros-args'], ['--radius', 'nan'], ['--session-seconds', '0'],
                                 ['--output-mode', '/cmd_vel'], ['--output-topic', '/cmd_vel']])
def test_adapter_rejects_remapping_and_invalid_arguments(args):
    with pytest.raises(SystemExit):
        parse_options(args)
