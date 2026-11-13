import pytest

from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, StopState, Velocity, ZERO


@pytest.mark.parametrize('field', list(Velocity.__dataclass_fields__))
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf'), None, 'bad', True, 10**400])
def test_invalid_component_clears_previous_command(field, value):
    policy = CommandPolicy(PolicyConfig())
    assert policy.accept(Velocity(linear_x=0.05))
    assert not policy.accept(Velocity(**{field: value}))
    assert policy.output() == ZERO
    assert policy.state == StopState.INVALID


@pytest.mark.parametrize('field', ['linear_z', 'angular_x', 'angular_y'])
@pytest.mark.parametrize('value', [-0.01, 0.01])
def test_unsupported_axis_is_rejected(field, value):
    policy = CommandPolicy(PolicyConfig())
    assert not policy.accept(Velocity(**{field: value}))
    assert policy.output() == ZERO


@pytest.mark.parametrize('command', [None, object(), [0] * 6])
def test_invalid_command_type(command):
    policy = CommandPolicy(PolicyConfig())
    assert not policy.accept(command)
    assert policy.state == StopState.INVALID


def test_valid_command_recovers_after_invalid_input():
    policy = CommandPolicy(PolicyConfig())
    policy.accept(Velocity(linear_z=1))
    assert policy.accept(Velocity(linear_x=0.02))
    assert policy.state == StopState.ACTIVE
    assert policy.output().linear_x == 0.02
