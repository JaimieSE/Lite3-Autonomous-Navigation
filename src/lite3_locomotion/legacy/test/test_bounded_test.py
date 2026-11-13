import pytest

from lite3_legacy.bounded_test import (
    BoundedMovementTest, MANUAL_MODE_CODE, TestPlan, TestState,
)
from lite3_locomotion.command_policy import ZERO


@pytest.fixture
def session():
    now = [0.0]
    return BoundedMovementTest(clock=lambda: now[0]), now


def assert_handback(intents):
    assert len(intents) == 2
    assert intents[0].kind == 'velocity'
    assert intents[0].velocity == ZERO
    assert intents[1].kind == 'manual_mode_request'
    assert intents[1].cmd_code == MANUAL_MODE_CODE == 0x21010C02


def test_disarmed_has_no_output(session):
    test, _ = session
    assert test.state == TestState.DISARMED
    assert test.tick() == ()


@pytest.mark.parametrize('navigation,operator', [(False, True), (True, False), (1, True), (True, 1)])
def test_explicit_preconditions_required(session, navigation, operator):
    test, _ = session
    with pytest.raises(ValueError):
        test.start(navigation_confirmed=navigation, operator_ready=operator)
    assert test.state == TestState.DISARMED
    assert test.tick() == ()


def test_short_forward_only_pulse_then_zero_and_manual(session):
    test, now = session
    first = test.start(navigation_confirmed=True, operator_ready=True)
    assert first[0].velocity.linear_x == 0.03
    for tick in (0.1, 0.2, 0.3, 0.4):
        now[0] = tick
        intents = test.tick()
        assert len(intents) == 1
        assert intents[0].velocity.linear_x == 0.03
        assert intents[0].velocity.linear_y == intents[0].velocity.angular_z == 0
    now[0] = 0.5
    assert_handback(test.tick())
    assert test.reason == 'duration_complete'
    assert test.state == TestState.HANDBACK


@pytest.mark.parametrize('gap', [0.2, 0.3, 10.0, -0.1])
def test_missed_update_or_clock_regression_stops_before_more_velocity(session, gap):
    test, now = session
    test.start(navigation_confirmed=True, operator_ready=True)
    now[0] = gap
    assert_handback(test.tick())
    assert test.reason == 'update_watchdog'


def test_kill_cannot_rearm_and_does_not_extend_handback_deadline(session):
    test, now = session
    test.start(navigation_confirmed=True, operator_ready=True)
    assert_handback(test.kill())
    now[0] = 1.0
    assert_handback(test.kill())
    assert_handback(test.tick())
    with pytest.raises(RuntimeError):
        test.start(navigation_confirmed=True, operator_ready=True)
    now[0] = 2.0
    assert_handback(test.tick())
    assert test.state == TestState.FAULT
    assert test.tick() == ()


def test_handback_requires_confirmation_and_never_produces_more_motion(session):
    test, now = session
    test.kill()
    with pytest.raises(ValueError):
        test.confirm_manual(False)
    now[0] = 0.2
    assert_handback(test.tick())
    test.confirm_manual(True)
    assert test.state == TestState.COMPLETE
    assert test.tick() == test.kill() == test.shutdown() == ()
    with pytest.raises(RuntimeError):
        test.start(navigation_confirmed=True, operator_ready=True)


def test_shutdown_and_prestart_kill_also_latch(session):
    test, _ = session
    assert_handback(test.shutdown())
    assert test.reason == 'shutdown'
    with pytest.raises(RuntimeError):
        test.start(navigation_confirmed=True, operator_ready=True)


def test_unexpected_manual_confirmation_is_not_accepted(session):
    test, _ = session
    with pytest.raises(ValueError):
        test.confirm_manual(True)


@pytest.mark.parametrize('field', ['speed', 'duration', 'update_timeout', 'handback_timeout'])
@pytest.mark.parametrize('value', [0, -1, float('nan'), float('inf'), True, None, 100, 10**400])
def test_invalid_or_unbounded_plans_rejected(field, value):
    with pytest.raises(ValueError):
        TestPlan(**{field: value})


def test_late_manual_confirmation_cannot_hide_a_handback_timeout(session):
    test, now = session
    test.kill()
    now[0] = 2.0
    with pytest.raises(ValueError):
        test.confirm_manual(True)
    assert test.state == TestState.FAULT
    assert test.tick() == ()
