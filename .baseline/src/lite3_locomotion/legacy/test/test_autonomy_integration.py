"""Real Nav2 with synthetic sensors; opt in through the loopback-only runner."""
from collections import deque
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('LITE3_RUN_AUTONOMY_INTEGRATION') != '1',
                                reason='Only the isolated Nav2 dry-run runner may start these processes')


class World:
    def __init__(self, output_mode='dry-run'):
        root = Path(__file__).resolve().parents[1]
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        assert 40 <= int(os.environ['ROS_DOMAIN_ID']) <= 89
        assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            assert os.environ[key] == str(root / 'test/loopback_dds.xml')
        assert 'ROS_DISCOVERY_SERVER' not in os.environ
        assert 'ROS_SUPER_CLIENT' not in os.environ
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import TransformStamped, Twist
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger
        from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.node = Node('synthetic_world', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        assert self.node.get_node_names_and_namespaces() == [('synthetic_world', '/')], 'Test domain occupied'
        self.trigger = Trigger
        self.twist = Twist
        self.transform = TransformStamped
        self.sensor_enabled = True
        self.scan_stamp_old = False
        self.obstacle = False
        self.status = {}
        self.mission = {}
        self.enabled = False
        self.samples = deque(maxlen=1000)
        self.raw = deque(maxlen=1000)
        self.static = StaticTransformBroadcaster(self.node)
        self.dynamic = TransformBroadcaster(self.node)
        static = TransformStamped()
        static.header.frame_id, static.child_frame_id = 'map', 'odom'
        static.header.stamp = self.node.get_clock().now().to_msg()
        static.transform.rotation.w = 1.
        self.static.sendTransform(static)
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.command_pub = self.node.create_publisher(Twist, '/lite3_autonomy/nav_cmd_vel', 1)
        self.subs = [
            self.node.create_subscription(Twist, '/lite3_locomotion/guarded_test_cmd_vel',
                                          lambda m: self.samples.append((time.monotonic(), m)), 10),
            self.node.create_subscription(Twist, '/lite3_autonomy/nav_cmd_vel',
                                          lambda m: self.raw.append((time.monotonic(), m)), 10),
            self.node.create_subscription(String, '/lite3_locomotion/guard_status', self.on_status, 10),
            self.node.create_subscription(String, '/lite3_locomotion/exploration_status', self.on_mission, 10),
            self.node.create_subscription(Bool, '/lite3_locomotion/autonomy_enabled',
                                          lambda m: setattr(self, 'enabled', m.data), 10),
        ]
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.resolution = 0.1
        self.map.info.width = self.map.info.height = 100
        self.map.info.origin.position.x = self.map.info.origin.position.y = -5.
        self.map.info.origin.orientation.w = 1.
        self.map.data = [0 if 30 <= x < 70 and 30 <= y < 70 else -1
                         for y in range(100) for x in range(100)]
        self.scan = LaserScan()
        self.scan.header.frame_id = 'base_link'
        self.scan.angle_min, self.scan.angle_max = -math.pi, math.pi
        self.scan.angle_increment = 2 * math.pi / 359
        self.scan.range_min, self.scan.range_max = 0.1, 10.
        self.odom = Odometry()
        self.odom.header.frame_id, self.odom.child_frame_id = 'odom', 'base_link'
        self.odom.pose.pose.orientation.w = 1.
        self.timer = self.node.create_timer(0.05, self.publish)
        self.map_timer = self.node.create_timer(0.5, self.publish_map)
        self.log = (Path(os.environ['LITE3_AUTONOMY_RESULTS']) / 'nav2.log').open('w')
        self.process = subprocess.Popen(['ros2', 'launch', 'lite3_locomotion', 'autonomy.launch.py',
                                         'output_mode:=' + output_mode], stdout=self.log, stderr=subprocess.STDOUT)

    def on_status(self, message):
        self.status = json.loads(message.data)

    def on_mission(self, message):
        self.mission = json.loads(message.data)

    def publish_map(self):
        self.map.header.stamp = self.node.get_clock().now().to_msg()
        self.map_pub.publish(self.map)

    def publish(self):
        if not self.sensor_enabled:
            return
        now = self.node.get_clock().now().to_msg()
        tf = self.transform()
        tf.header.frame_id, tf.child_frame_id = 'odom', 'base_link'
        tf.header.stamp = now
        tf.transform.rotation.w = 1.
        self.dynamic.sendTransform(tf)
        self.odom.header.stamp = now
        self.odom_pub.publish(self.odom)
        if not self.scan_stamp_old:
            self.scan.header.stamp = now
        self.scan.ranges = [0.2 if self.obstacle else 4.] * 360
        self.scan_pub.publish(self.scan)

    def wait(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.process.poll() is None, 'Autonomy launch exited; inspect nav2.log'
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        pytest.fail(f'Condition timed out; guard={self.status}, mission={self.mission}; inspect nav2.log')

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def call(self, name):
        client = self.node.create_client(self.trigger, '/lite3_locomotion/' + name)
        try:
            self.wait(client.service_is_ready)
            future = client.call_async(self.trigger.Request())
            self.wait(future.done)
            response = future.result()
            assert response.success, response.message
        finally:
            self.node.destroy_client(client)

    def send(self, x=0.04, yaw=0.):
        command = self.twist()
        command.linear.x, command.angular.z = float(x), float(yaw)
        self.command_pub.publish(command)

    def control(self, command):
        process = subprocess.Popen(['ros2', 'run', 'lite3_locomotion', 'autonomy_control', command],
                                   stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            self.wait(lambda: process.poll() is not None, 12)
            output = process.communicate()[0]
            assert process.returncode == 0, output
            print(output)
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3)
        self.log.close()
        self.executor.shutdown()
        self.node.destroy_node()
        self.rclpy.shutdown(context=self.context)


@pytest.fixture(scope='module')
def world():
    world = World()
    try:
        world.wait(lambda: world.status.get('health') == 'healthy', 25)
        from rclpy.action import ActionClient
        from nav2_msgs.action import NavigateToPose
        client = ActionClient(world.node, NavigateToPose, '/lite3_autonomy/navigate_to_pose')
        world.wait(client.server_is_ready, 25)
        world.wait(lambda: world.mission.get('nav2_active'), 25)
        client.destroy()
        yield world
    finally:
        world.close()


def moving(message):
    return abs(message.linear.x) + abs(message.linear.y) + abs(message.angular.z) > 1e-6


def test_startup_disarmed_and_no_robot_outputs(world):
    world.spin(0.5)
    assert not world.enabled
    assert world.status['output_mode'] == 'dry-run'
    world.send(1, 1)
    world.spin(0.2)
    assert world.samples and not any(moving(m) for _, m in world.samples)
    topics = dict(world.node.get_topic_names_and_types())
    assert '/cmd_vel' not in topics and '/cmd_vel_out' not in topics
    assert '/lite3_robot_link/cmd_vel' not in topics
    assert not any('jetson2motion' in name for name in topics)


def test_guard_limits_stale_latch_and_explicit_stop(world):
    world.call('arm')
    world.send(10, -10)
    world.wait(lambda: any(moving(m) for _, m in world.samples), 1)
    message = next(m for _, m in reversed(world.samples) if moving(m))
    assert message.linear.x == 0.1 and message.angular.z == -0.2
    world.wait(lambda: world.status.get('reason') == 'command_timeout' and not world.enabled, 2)
    assert not world.enabled
    world.send(0.04)
    world.spin(0.2)
    assert not moving(world.samples[-1][1])
    world.call('arm')
    world.send()
    world.wait(lambda: moving(world.samples[-1][1]), 1)
    world.call('stop')
    world.wait(lambda: not world.enabled and not moving(world.samples[-1][1]), 1)


def test_real_nav2_frontier_goal_to_dry_run_and_cancel(world):
    world.call('arm')
    world.spin(0.15)
    world.call('start_exploration')
    began = time.monotonic()
    world.wait(lambda: any(t >= began and moving(m) for t, m in world.samples), 5)
    assert world.enabled
    assert any(t >= began and moving(m) for t, m in world.raw)
    for t, m in world.samples:
        if t >= began:
            assert abs(m.linear.x) <= 0.1 and m.linear.y == 0 and abs(m.angular.z) <= 0.2
    world.call('stop')
    world.call('stop_exploration')
    world.wait(lambda: not world.enabled and not moving(world.samples[-1][1]), 1)
    world.spin(0.6)
    assert not world.enabled
    assert all(not moving(m) for t, m in world.samples if t >= time.monotonic() - 0.3)


def test_obstacle_and_stale_header_latch(world):
    world.call('arm')
    world.send()
    world.wait(lambda: moving(world.samples[-1][1]), 1)
    world.obstacle = True
    world.wait(lambda: not world.enabled, 1)
    assert not moving(world.samples[-1][1])
    world.obstacle = False
    world.wait(lambda: world.status.get('health') == 'healthy', 1)
    world.call('arm')
    world.send()
    world.scan_stamp_old = True
    # Keep commands and receipt times fresh: only the scan's header stops advancing.
    deadline = time.monotonic() + 0.8
    while time.monotonic() < deadline:
        world.send()
        world.spin(0.05)
    assert not world.enabled
    assert not moving(world.samples[-1][1])
    world.scan_stamp_old = False


def test_cli_start_and_stop(world):
    world.wait(lambda: world.status.get('health') == 'healthy', 1)
    world.samples.clear()
    world.control('start')
    world.wait(lambda: any(moving(m) for _, m in world.samples), 3)
    world.control('stop')
    world.wait(lambda: not world.enabled and not moving(world.samples[-1][1]), 1)
    world.spin(0.5)
    assert not world.enabled
    assert all(not moving(m) for t, m in world.samples if t >= time.monotonic() - 0.3)


def test_sensor_stream_loss_stops_despite_fresh_commands(world):
    world.call('arm')
    world.send()
    world.wait(lambda: moving(world.samples[-1][1]), 1)
    world.sensor_enabled = False
    deadline = time.monotonic() + 0.8
    try:
        while time.monotonic() < deadline:
            world.send()
            world.spin(0.05)
        assert not world.enabled
        assert not moving(world.samples[-1][1])
    finally:
        world.sensor_enabled = True
