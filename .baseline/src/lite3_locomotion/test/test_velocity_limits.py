import pytest

from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, Velocity


@pytest.mark.parametrize('sign', [-1, 1])
def test_all_supported_axes_are_clamped_symmetrically(sign):
    policy = CommandPolicy(PolicyConfig())
    assert policy.accept(Velocity(linear_x=sign * 2, linear_y=sign * 2, angular_z=sign * 2))
    assert policy.output() == Velocity(
        linear_x=sign * 0.10, linear_y=sign * 0.05, angular_z=sign * 0.20,
    )


def test_in_range_values_are_preserved():
    policy = CommandPolicy(PolicyConfig())
    command = Velocity(linear_x=0.02, linear_y=-0.03, angular_z=0.1)
    assert policy.accept(command)
    assert policy.output() == command


def test_custom_limits_and_disabled_axis():
    policy = CommandPolicy(PolicyConfig(max_linear_x=0.03, max_linear_y=0, max_angular_z=0.04))
    policy.accept(Velocity(linear_x=0.1, linear_y=-0.1, angular_z=-0.1))
    assert policy.output() == Velocity(linear_x=0.03, angular_z=-0.04)


@pytest.mark.parametrize('field', list(PolicyConfig.__dataclass_fields__))
@pytest.mark.parametrize('value', [-1, float('nan'), float('inf'), None, True])
def test_invalid_configuration(field, value):
    with pytest.raises(ValueError):
        PolicyConfig(**{field: value})


def test_timeout_cannot_be_disabled():
    with pytest.raises(ValueError):
        PolicyConfig(command_timeout=0)
