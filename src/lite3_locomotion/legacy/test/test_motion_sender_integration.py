"""Actual sender executable and UDP receiver; isolated opt-in runner only."""

import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('LITE3_RUN_SENDER_INTEGRATION') != '1',
                                reason='Use isolated loopback sender runner')


class World:
    def __init__(self):
        root = Path(__file__).resolve().parents[1]
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        assert 40 <= int(os.environ['ROS_DOMAIN_ID']) <= 89
        assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        assert 'ROS_DISCOVERY_SERVER' not in os.environ
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            assert os.environ[key] == str(root / 'test/loopback_dds.xml')
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        self.rclpy, self.Twist, self.Trigger = rclpy, Twist, Trigger
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node('lite3_guarded_adapter', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.spin(0.5)
        assert self.node.get_node_names_and_namespaces() == [('lite3_guarded_adapter', '/')], 'Domain occupied'
        self.pub = self.node.create_publisher(Twist, '/lite3_robot_link/cmd_vel', 1)
        self.String = String
        self.ack_pub = self.node.create_publisher(String, '/lite3_robot_link/session_ack', 1)
        self.status = {}
        self.sub = self.node.create_subscription(String, '/lite3_robot_link/status',
                                                 lambda m: setattr(self, 'status', json.loads(m.data)), 10)
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(('127.0.0.1', 0))
        self.udp.setblocking(False)
        self.log = (Path(os.environ['LITE3_SENDER_RESULTS']) / 'sender.log').open('w')
        self.process = subprocess.Popen([
            str(root / '.colcon/install/lite3_locomotion/lib/lite3_locomotion/motion_sender'),
            '--target-ip', '127.0.0.1', '--target-port', str(self.udp.getsockname()[1])],
            stdout=self.log, stderr=subprocess.STDOUT)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def wait(self, predicate, seconds=5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'Sender exited; inspect sender.log'
            self.spin(0.02)
            if predicate():
                return
        pytest.fail('Timed out: ' + str(self.status))

    def call(self, command):
        client = self.node.create_client(self.Trigger, '/lite3_robot_link/' + command)
        try:
            self.wait(client.service_is_ready)
            future = client.call_async(self.Trigger.Request())
            self.wait(future.done)
            return future.result()
        finally:
            self.node.destroy_client(client)

    def send(self, x=0.02, yaw=0.0, lateral=0.0):
        msg = self.Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(x), float(lateral), float(yaw)
        self.pub.publish(msg)

    def arm(self):
        old_session = self.status.get('session_id')
        assert self.call('enable').success
        self.wait(lambda: self.status.get('enabled') and self.status.get('session_id') != old_session)
        ack = self.String()
        ack.data = self.status['session_id']
        self.ack_pub.publish(ack)
        self.wait(lambda: self.status.get('authorized'))

    def drain(self):
        packets = []
        while True:
            try:
                data, _ = self.udp.recvfrom(100)
                assert len(data) == 20
                packets.append(struct.unpack('<iiid', data))
            except BlockingIOError:
                return packets

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)
        self.log.close()
        self.udp.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.try_shutdown()


@pytest.fixture(scope='module')
def world():
    instance = World()
    try:
        instance.wait(lambda: bool(instance.status))
        yield instance
    finally:
        instance.close()


def test_startup_silent_and_unarmed_input_ignored(world):
    world.send()
    world.spin(0.3)
    assert not world.status['enabled']
    assert world.drain() == []


def test_enable_limits_stop_and_expiry_latch(world):
    world.arm()
    world.send(10, -10)
    world.spin(0.15)
    packets = world.drain()
    assert (320, 8, 1, 0.03) in packets
    assert (325, 8, 1, 0.0) in packets
    assert (321, 8, 1, 0.10) in packets
    world.wait(lambda: not world.status['enabled'])
    assert world.status['reason'] == 'command_timeout'
    world.spin(0.2)
    world.drain()
    world.send()
    world.spin(0.15)
    assert all(p[3] == 0 for p in world.drain())
    world.arm()
    world.send()
    world.spin(0.1)
    assert any(p[0] == 320 and p[3] == 0.02 for p in world.drain())
    assert world.call('stop').success
    world.spin(0.15)
    assert all(p[3] == 0 for p in world.drain())


def test_invalid_command_latches_and_unexpected_publisher_rejects_enable(world):
    world.arm()
    world.send(lateral=0.01)
    world.wait(lambda: world.status.get('reason') == 'invalid_command')
    assert world.status['reason'] == 'invalid_command'
    extra = world.node.create_publisher(world.Twist, '/lite3_robot_link/cmd_vel', 1)
    try:
        world.spin(0.3)
        assert not world.call('enable').success
    finally:
        world.node.destroy_publisher(extra)


def test_shutdown_during_motion_attempts_zero_and_exits(world):
    world.spin(0.3)
    world.arm()
    world.send()
    world.spin(0.1)
    assert any(p[0] == 320 and p[3] == 0.02 for p in world.drain())
    world.process.send_signal(signal.SIGINT)
    world.process.wait(timeout=4)
    assert world.process.returncode == 0
    packets = world.drain()
    assert len(packets) >= 3
    assert all(p[3] == 0 for p in packets[-3:])
