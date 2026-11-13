import pytest

from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, StopState, Velocity, ZERO


def test_startup_is_zero_until_first_command():
    policy = CommandPolicy(PolicyConfig())
    assert policy.output() == ZERO
    assert policy.state == StopState.WAITING


def test_stop_latches_and_release_requires_new_command():
    policy = CommandPolicy(PolicyConfig())
    command = Velocity(linear_x=0.03)
    policy.accept(command)
    policy.set_stop(True)
    assert policy.output() == ZERO
    assert policy.state == StopState.STOPPED
    assert not policy.accept(command)
    assert not policy.accept(Velocity(linear_x=float('nan')))
    assert policy.state == StopState.STOPPED
    policy.set_stop(False)
    assert policy.output() == ZERO
    assert policy.state == StopState.WAITING
    assert policy.accept(command)
    assert policy.output() == command


def test_stop_is_idempotent_and_inactive_clear_does_not_erase_command():
    policy = CommandPolicy(PolicyConfig())
    policy.set_stop(True)
    policy.set_stop(True)
    policy.set_stop(False)
    command = Velocity(linear_x=0.03)
    policy.accept(command)
    policy.set_stop(False)
    assert policy.output() == command


def test_stop_rejects_ambiguous_flag():
    policy = CommandPolicy(PolicyConfig())
    with pytest.raises(ValueError):
        policy.set_stop('false')
