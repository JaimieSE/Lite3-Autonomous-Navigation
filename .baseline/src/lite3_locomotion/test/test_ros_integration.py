"""Actual DDS tests. Opt in only through tools/run_ros_integration.py."""

from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


pytestmark = pytest.mark.skipif(
    os.environ.get('LITE3_RUN_ROS_INTEGRATION') != '1',
    reason='Requires explicit isolated runner; ordinary unit tests never start ROS',
)

ROOT = Path(__file__).resolve().parents[1]
INPUT = '/lite3_locomotion/test_nav_cmd_vel'
OUTPUT = '/lite3_locomotion/test_cmd_vel'
KILL = '/lite3_locomotion/test_kill'
RESET = '/lite3_locomotion/test_reset_kill'
CONTROLLER = 'lite3_locomotion_controller'
OBSERVER = 'lite3_locomotion_integration_observer'


def values(message):
    return (
        message.linear.x, message.linear.y, message.linear.z,
        message.angular.x, message.angular.y, message.angular.z,
    )


class Harness:
    def __init__(self, node, executor, process, twist):
        from std_srvs.srv import Trigger

        self.node = node
        self.executor = executor
        self.process = process
        self.twist = twist
        self.discovering = True
        self.samples = deque(maxlen=200)
        self.trigger = Trigger
        self.clients = {name: node.create_client(Trigger, name) for name in (KILL, RESET)}
        self.publisher = node.create_publisher(twist, INPUT, 1)
        self.subscription = node.create_subscription(
            twist, OUTPUT, lambda msg: self.samples.append((time.monotonic(), values(msg))), 10,
        )

    def guard(self):
        assert self.process.poll() is None, 'Controller exited unexpectedly'
        names = self.node.get_node_names_and_namespaces()
        assert all(ns == '/' and name in (OBSERVER, CONTROLLER) for name, ns in names), (
            f'Unexpected ROS nodes in test domain: {names}'
        )
        ready = sorted(names) == sorted([(OBSERVER, '/'), (CONTROLLER, '/')])
        for topic, publisher_name, subscriber_name in (
            (INPUT, OBSERVER, CONTROLLER), (OUTPUT, CONTROLLER, OBSERVER),
        ):
            for endpoints, expected in (
                (self.node.get_publishers_info_by_topic(topic), publisher_name),
                (self.node.get_subscriptions_info_by_topic(topic), subscriber_name),
            ):
                ready = ready and len(endpoints) == 1
                assert len(endpoints) <= 1, f'Unexpected extra endpoints on {topic}'
                for endpoint in endpoints:
                    owner = (endpoint.node_name, endpoint.node_namespace)
                    if self.discovering and owner == ('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_'):
                        ready = False  # Discovery is incomplete; no command publication allowed.
                        continue
                    assert owner == (expected, '/'), f'Unexpected endpoint owner: {owner}'
        for topic, _ in self.node.get_topic_names_and_types():
            assert topic in (INPUT, OUTPUT, '/parameter_events'), f'Unexpected topic: {topic}'
        if not self.discovering:
            assert ready, 'Test endpoints disappeared or ownership is incomplete'
        return ready

    def spin(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.guard()
            self.executor.spin_once(timeout_sec=min(0.02, max(0, deadline - time.monotonic())))

    def wait_for(self, predicate, timeout=3):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            self.spin(0.02)
        raise AssertionError('Timed out waiting for expected DDS observation')

    def command(self, vx=0.0, vy=0.0, wz=0.0, *, vz=0.0):
        message = self.twist()
        message.linear.x, message.linear.y, message.linear.z = vx, vy, vz
        message.angular.z = wz
        assert not self.discovering and self.guard()
        self.samples.clear()
        self.publisher.publish(message)
        return time.monotonic()

    def expect(self, expected, timeout=0.4):
        self.wait_for(
            lambda: any(sample == pytest.approx(expected) for _, sample in self.samples),
            timeout=timeout,
        )

    def call(self, name):
        assert not self.discovering and self.guard()
        client = self.clients[name]
        self.wait_for(client.service_is_ready)
        self.samples.clear()
        future = client.call_async(self.trigger.Request())
        self.wait_for(future.done)
        assert future.result().success


@pytest.fixture(scope='module')
def harness():
    profile = str(ROOT / 'test' / 'loopback_dds.xml')
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert 40 <= int(os.environ['ROS_DOMAIN_ID']) <= 89
    assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] == profile
    assert os.environ['FASTDDS_DEFAULT_PROFILES_FILE'] == profile
    assert 'ROS_DISCOVERY_SERVER' not in os.environ
    assert 'ROS_SUPER_CLIENT' not in os.environ
    executable = ROOT / '.colcon/audit_install/lite3_locomotion/lib/lite3_locomotion/locomotion_controller'
    assert os.environ['LITE3_TEST_EXECUTABLE'] == str(executable)

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node

    # Verify the installed package is tested, rather than implicitly importing source.
    import lite3_locomotion
    assert (ROOT / '.colcon/audit_install') in Path(lite3_locomotion.__file__).resolve().parents
    context = Context()
    rclpy.init(args=[], context=context)
    node = None
    executor = None
    process = None
    log = None
    try:
        node = Node(
            OBSERVER, namespace='/', context=context, use_global_arguments=False,
            enable_rosout=False, start_parameter_services=False,
        )
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        # Discover first: reject an occupied domain before starting the controller.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            assert all(
                entry == (OBSERVER, '/') for entry in node.get_node_names_and_namespaces()
            ), 'Test domain is occupied; aborting before controller startup'
        log = (ROOT / '.colcon/integration/controller.log').open('w')
        process = subprocess.Popen(
            [sys.executable, str(executable)], stdout=log, stderr=subprocess.STDOUT,
        )
        test = Harness(node, executor, process, Twist)
        test.wait_for(lambda: (
            test.publisher.get_subscription_count() == 1 and
            node.count_publishers(OUTPUT) == 1 and bool(test.samples) and test.guard()
        ), timeout=8)
        test.discovering = False
        yield test
    finally:
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3)
        if log is not None:
            log.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


def test_startup_zero_and_output_cadence(harness):
    harness.samples.clear()
    harness.spin(0.65)
    assert len(harness.samples) >= 4
    assert all(sample == (0.0,) * 6 for _, sample in harness.samples)


def test_valid_command_passes_through(harness):
    harness.command(0.04, -0.02, 0.08)
    harness.expect((0.04, -0.02, 0, 0, 0, 0.08))


@pytest.mark.parametrize('sign', [-1.0, 1.0])
def test_excessive_command_is_limited(harness, sign):
    harness.command(sign, sign, sign)
    harness.expect((sign * 0.1, sign * 0.05, 0, 0, 0, sign * 0.2))


@pytest.mark.parametrize('invalid', ['nan', 'unsupported_axis'])
def test_invalid_input_clears_active_command(harness, invalid):
    harness.command(0.03)
    harness.expect((0.03, 0, 0, 0, 0, 0))
    start = harness.command(float('nan') if invalid == 'nan' else 0.03,
                            vz=0.01 if invalid == 'unsupported_axis' else 0.0)
    harness.expect((0.0,) * 6, timeout=0.35)  # Before normal command expiry.
    zero_at = next(t for t, sample in harness.samples if sample == (0.0,) * 6)
    assert zero_at - start < 0.35


def test_timeout_zero_persists_and_fresh_input_recovers(harness):
    start = harness.command(0.06)
    harness.expect((0.06, 0, 0, 0, 0, 0))
    # Discard pre-command or previously queued zeros before timing expiry.
    harness.samples.clear()
    harness.spin(0.85)
    zeros = [t for t, sample in harness.samples if sample == (0.0,) * 6]
    assert zeros, 'No zero fallback observed'
    # Receipt-to-expiry is 0.5s; allow DDS observation and timer scheduling margin.
    assert 0.45 <= zeros[0] - start <= 0.85
    first_zero = zeros[0]
    assert all(sample == (0.0,) * 6 for t, sample in harness.samples if t >= first_zero)
    harness.command(-0.02)
    harness.expect((-0.02, 0, 0, 0, 0, 0))


@pytest.mark.parametrize('target', ['/cmd_vel', '/cmd_vel_out'])
def test_executable_rejects_remapping(harness, target):
    executable = os.environ['LITE3_TEST_EXECUTABLE']
    result = subprocess.run(
        [sys.executable, executable, '--ros-args', '-r', OUTPUT + ':=' + target],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode == 2
    assert 'unrecognized arguments' in result.stderr
    harness.guard()
    assert harness.node.count_publishers(target) == 0


def test_kill_service_latches_zero_despite_continuing_commands(harness):
    harness.command(0.04)
    harness.expect((0.04, 0, 0, 0, 0, 0))
    harness.call(KILL)
    harness.expect((0.0,) * 6)
    harness.call(KILL)  # Repeated kill is safe and never toggles autonomy on.
    for _ in range(4):
        harness.command(0.04)
        harness.spin(0.15)
        assert harness.samples
        assert all(sample == (0.0,) * 6 for _, sample in harness.samples)


def test_reset_kill_does_not_replay_old_commands(harness):
    harness.call(KILL)
    harness.command(0.05)
    harness.spin(0.15)
    harness.call(RESET)
    harness.samples.clear()
    harness.spin(0.25)
    assert harness.samples
    assert all(sample == (0.0,) * 6 for _, sample in harness.samples)
    harness.command(0.02)
    harness.expect((0.02, 0, 0, 0, 0, 0))
