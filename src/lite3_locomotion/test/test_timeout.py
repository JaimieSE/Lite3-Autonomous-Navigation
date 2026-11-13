from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, StopState, Velocity, ZERO


def test_timeout_boundary_and_persistent_zero():
    now = [0.0]
    policy = CommandPolicy(PolicyConfig(), clock=lambda: now[0])
    command = Velocity(linear_x=0.04)
    policy.accept(command)
    now[0] = 0.499
    assert policy.output() == command
    now[0] = 0.5
    assert policy.output() == ZERO
    assert policy.state == StopState.TIMED_OUT
    now[0] = 100
    assert policy.output() == ZERO


def test_fresh_command_refreshes_deadline_and_recovers_timeout():
    now = [0.0]
    policy = CommandPolicy(PolicyConfig(command_timeout=1), clock=lambda: now[0])
    command = Velocity(linear_x=0.04)
    policy.accept(command)
    now[0] = 0.75
    policy.accept(command)
    now[0] = 1.5
    assert policy.output() == command
    now[0] = 1.75
    assert policy.output() == ZERO
    assert policy.accept(command)
    assert policy.output() == command


def test_invalid_command_does_not_refresh_or_restore_previous_command():
    now = [0.0]
    policy = CommandPolicy(PolicyConfig(), clock=lambda: now[0])
    policy.accept(Velocity(linear_x=0.04))
    now[0] = 0.4
    assert not policy.accept(Velocity(angular_z=float('nan')))
    assert policy.output() == ZERO
    now[0] = 0.6
    assert policy.output() == ZERO
    assert policy.state == StopState.INVALID
