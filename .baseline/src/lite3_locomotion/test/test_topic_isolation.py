"""Exercise the ROS adapter with fakes, without initializing DDS or ROS."""

import sys
from types import ModuleType, SimpleNamespace

import pytest

from lite3_locomotion.command_policy import PolicyConfig
from lite3_locomotion.controller_node import (
    INPUT_TOPIC, OUTPUT_TOPIC, KILL_SERVICE, RESET_KILL_SERVICE,
    create_controller_node, main, parse_config,
)


@pytest.mark.parametrize('args', [
    ['--ros-args', '-r', '__ns:=/robot'],
    ['--ros-args', '-r', OUTPUT_TOPIC + ':=/cmd_vel'],
    ['--ros-args', '-r', OUTPUT_TOPIC + ':=/cmd_vel_out'],
    ['--ros-args', '--params-file', 'robot.yaml'],
    ['--ros-args', '-r', KILL_SERVICE + ':=/simple_cmd'],
    ['--output-topic', '/cmd_vel'],
    ['-r', OUTPUT_TOPIC + ':=/cmd_vel'],
    ['--max-linear-x', 'nan'],
    ['--command-timeout', '0'],
    ['--max-l', '0.1'],
])
def test_unsafe_or_invalid_arguments_fail_before_ros_initialization(args, monkeypatch):
    # ROS must not even be imported for these entry-point failures.
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2


def test_application_arguments_only_configure_policy():
    assert parse_config(['--max-linear-x', '0.03', '--command-timeout', '0.2']) == PolicyConfig(
        max_linear_x=0.03, command_timeout=0.2,
    )


@pytest.fixture
def fake_ros(monkeypatch):
    calls = SimpleNamespace(options=None, published=[], subscriptions=[], publishers=[],
                            services={}, destroyed=False)

    class Twist:
        def __init__(self):
            self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
            self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)

    class Node:
        def __init__(self, name, **kwargs):
            calls.options = kwargs

        def resolve_topic_name(self, name):
            return name

        def resolve_service_name(self, name):
            return name

        def create_service(self, service_type, name, callback):
            calls.services[name] = callback
            return callback

        def create_publisher(self, message_type, topic, depth):
            calls.publishers.append(topic)
            return SimpleNamespace(publish=calls.published.append)

        def create_subscription(self, message_type, topic, callback, depth):
            calls.subscriptions.append(topic)
            return callback

        def create_timer(self, period, callback):
            return callback

        def get_logger(self):
            return SimpleNamespace(info=lambda message: None, warning=lambda message: None)

        def destroy_node(self):
            calls.destroyed = True

    for name, attributes in [
        ('rclpy', {}), ('rclpy.node', {'Node': Node}),
        ('geometry_msgs', {}), ('geometry_msgs.msg', {'Twist': Twist}),
        ('std_srvs', {}), ('std_srvs.srv', {'Trigger': object}),
    ]:
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    return calls, Node, Twist


def test_adapter_uses_fixed_topics_and_ignores_global_arguments(fake_ros):
    calls, _, Twist = fake_ros
    context = object()
    node = create_controller_node(PolicyConfig(), context)
    assert calls.options == dict(
        namespace='/', context=context, use_global_arguments=False,
        enable_rosout=False, start_parameter_services=False,
    )
    assert calls.publishers == [OUTPUT_TOPIC]
    assert calls.subscriptions == [INPUT_TOPIC]
    assert set(calls.services) == {KILL_SERVICE, RESET_KILL_SERVICE}
    node._publish_output()
    assert calls.published[-1].linear.x == 0
    message = Twist()
    message.linear.x = 1.0
    node._on_command(message)
    node._publish_output()
    assert calls.published[-1].linear.x == 0.10
    message.linear.z = 1.0
    node._on_command(message)
    assert calls.published[-1].linear.x == 0
    node.stop()
    assert calls.published[-1].angular.z == 0


def test_resolved_topic_mismatch_fails_before_creating_endpoints(fake_ros, monkeypatch):
    calls, Node, _ = fake_ros
    monkeypatch.setattr(Node, 'resolve_topic_name', lambda self, name: '/cmd_vel')
    with pytest.raises(RuntimeError, match='isolation'):
        create_controller_node(PolicyConfig(), object())
    assert calls.destroyed
    assert calls.publishers == []
    assert calls.subscriptions == []


def test_service_name_mismatch_fails_before_any_endpoints(fake_ros, monkeypatch):
    calls, Node, _ = fake_ros
    monkeypatch.setattr(Node, 'resolve_service_name', lambda self, name: '/wrong_service')
    with pytest.raises(RuntimeError, match='isolation'):
        create_controller_node(PolicyConfig(), object())
    assert calls.destroyed
    assert calls.publishers == []
    assert calls.subscriptions == []
    assert calls.services == {}


def test_kill_service_latches_and_reset_requires_fresh_input(fake_ros):
    calls, _, Twist = fake_ros
    node = create_controller_node(PolicyConfig(), object())
    command = Twist()
    command.linear.x = 0.04
    node._on_command(command)
    node._publish_output()
    assert calls.published[-1].linear.x == 0.04
    for _ in range(2):
        response = calls.services[KILL_SERVICE](None, SimpleNamespace())
        assert response.success
        assert calls.published[-1].linear.x == 0.0
        node._on_command(command)
        node._publish_output()
        assert calls.published[-1].linear.x == 0.0
    response = calls.services[RESET_KILL_SERVICE](None, SimpleNamespace())
    assert response.success
    assert calls.published[-1].linear.x == 0.0
    node._on_command(command)
    node._publish_output()
    assert calls.published[-1].linear.x == 0.04


@pytest.mark.parametrize('shutdown_early', [False, True])
def test_entry_point_owns_context_and_executor_and_cleans_up(monkeypatch, shutdown_early):
    import lite3_locomotion.controller_node as controller

    events = []

    class Context:
        active = True

        def ok(self):
            return self.active

    context = Context()
    node = SimpleNamespace(
        stop=lambda: events.append('stop'),
        destroy_node=lambda: events.append('destroy'),
    )

    class ExternalShutdownException(Exception):
        pass

    class Executor:
        def __init__(self, *, context):
            assert context is expected_context

        def add_node(self, value):
            assert value is node

        def spin(self):
            if shutdown_early:
                context.active = False
                raise ExternalShutdownException()
            raise KeyboardInterrupt()

        def shutdown(self):
            events.append('executor_shutdown')

    expected_context = context

    def init(*, args, context):
        assert args == []
        assert context is expected_context
        events.append('init')

    def shutdown(*, context):
        assert context is expected_context
        events.append('context_shutdown')

    def create(config, context):
        assert context is expected_context
        return node

    for name, attributes in [
        ('rclpy', {'init': init, 'shutdown': shutdown}),
        ('rclpy.context', {'Context': lambda: context}),
        ('rclpy.executors', {
            'ExternalShutdownException': ExternalShutdownException,
            'SingleThreadedExecutor': Executor,
        }),
    ]:
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(controller, 'create_controller_node', create)
    main([])
    if shutdown_early:
        assert events == ['init', 'executor_shutdown', 'destroy']
    else:
        assert events == ['init', 'stop', 'executor_shutdown', 'destroy', 'context_shutdown']
