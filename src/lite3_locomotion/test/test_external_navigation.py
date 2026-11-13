"""Existing NavigateToPose contract, synthetic sensors, no Nav2 or transport.

Only tools/run_autonomy_integration.py may opt into this real-DDS suite.
"""
import math
import os
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('LITE3_RUN_EXTERNAL_INTEGRATION') != '1',
                               reason='Use the isolated external-navigation runner')
_world_count = 0
_base_domain = int(os.environ.get('ROS_DOMAIN_ID', '40'))


class World:
    def __init__(self, dispatch=True, namespace=''):
        global _world_count
        root = Path(__file__).resolve().parents[1]
        assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
        assert 40 <= int(os.environ['ROS_DOMAIN_ID']) <= 89
        assert os.environ['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
        for key in ('FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            assert os.environ[key] == str(root / 'test/loopback_dds.xml')
        assert 'ROS_DISCOVERY_SERVER' not in os.environ
        # Separate DDS participants from earlier test contexts whose discovery
        # teardown may still be propagating. Every domain is checked empty.
        os.environ['ROS_DOMAIN_ID'] = str(40 + (_base_domain - 40 + _world_count) % 50)
        _world_count += 1
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
        from rclpy.action import ActionServer, GoalResponse, CancelResponse
        from rclpy.task import Future
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan
        from geometry_msgs.msg import TransformStamped
        from lifecycle_msgs.srv import GetState
        from tf2_ros import TransformBroadcaster
        from lite3_locomotion.exploration_coordinator import create_node, parse_options
        self.rclpy, self.Future, self.NavigateToPose = rclpy, Future, NavigateToPose
        self.context = Context()
        rclpy.init(args=[], context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.node = Node('mock_university_navigation', context=self.context, enable_rosout=False)
        self.executor.add_node(self.node)
        try:
            self.wait(lambda: self.node.get_node_names_and_namespaces() ==
                      [('mock_university_navigation', '/')], timeout=4)
        except Exception:
            self.executor.shutdown()
            self.node.destroy_node()
            rclpy.shutdown(context=self.context)
            raise
        self.accept_goals = self.accept_cancel = self.active = self.sensors = True
        self.map_enabled = True
        self.obstacle = self.stale_stamp = False
        self.mode = 'hold'
        self.goals, self.executing = [], []
        self.cancel_count = 0
        self.services = []
        for name in ('controller_server', 'planner_server', 'bt_navigator'):
            self.services.append(self.node.create_service(GetState, namespace + '/' + name + '/get_state',
                                                         self.lifecycle))
        self.server = ActionServer(self.node, NavigateToPose, namespace + '/navigate_to_pose',
                                   execute_callback=self.execute,
                                   goal_callback=lambda _: GoalResponse.ACCEPT if self.accept_goals else GoalResponse.REJECT,
                                   cancel_callback=self.cancel)
        self.CancelResponse = CancelResponse
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.node.create_publisher(OccupancyGrid, '/map', qos)
        self.scan_pub = self.node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.tf = TransformBroadcaster(self.node)
        self.TransformStamped = TransformStamped
        self.map = OccupancyGrid()
        self.map.header.frame_id = 'map'
        self.map.info.width = self.map.info.height = 30
        self.map.info.resolution = 0.2
        self.map.info.origin.position.x = self.map.info.origin.position.y = -3.0
        self.map.info.origin.orientation.w = 1.0
        self.map.data = [0 if 5 <= x < 25 and 5 <= y < 25 else -1
                         for y in range(30) for x in range(30)]
        for x, y in ((5, 5), (24, 5), (5, 24), (24, 24)):
            self.map.data[y * 30 + x] = 100  # four distinct frontier clusters
        self.scan = LaserScan()
        self.scan.header.frame_id = 'base_link'
        self.scan.angle_min, self.scan.angle_max = -math.pi, math.pi
        self.scan.angle_increment = 2 * math.pi / 359
        self.scan.range_min, self.scan.range_max = 0.1, 10.0
        args = ['--nav2-namespace', namespace] + (['--enable-navigation'] if dispatch else [])
        self.coordinator = create_node(parse_options(args), self.context)
        self.executor.add_node(self.coordinator)
        self.coordinator.timer.cancel()
        self.coordinator.create_timer(0.1, self.coordinator.tick)
        self.timer = self.node.create_timer(0.05, self.publish)
        self.wait(lambda: self.coordinator.ready() and self.coordinator.pose() is not None)

    def lifecycle(self, request, response):
        response.current_state.id = 3 if self.active else 2
        return response

    def cancel(self, handle):
        self.cancel_count += 1
        return self.CancelResponse.ACCEPT if self.accept_cancel else self.CancelResponse.REJECT

    async def execute(self, handle):
        self.goals.append(handle.request)
        future = self.Future()
        self.executing.append((handle, future))
        return await future

    def publish(self):
        now = self.node.get_clock().now().to_msg()
        if self.map_enabled:
            self.map.header.stamp = now
            self.map_pub.publish(self.map)
        if self.sensors:
            transform = self.TransformStamped()
            transform.header.frame_id, transform.child_frame_id = 'map', 'base_link'
            transform.header.stamp = now
            transform.transform.rotation.w = 1.0
            self.tf.sendTransform(transform)
            if not self.stale_stamp:
                self.scan.header.stamp = now
            self.scan.ranges = [0.2 if self.obstacle else 4.0] * 360
            self.scan_pub.publish(self.scan)
        for handle, future in list(self.executing):
            if handle.is_cancel_requested:
                handle.canceled()
            elif self.mode == 'success':
                handle.succeed()
            elif self.mode == 'abort':
                handle.abort()
            else:
                continue
            future.set_result(self.NavigateToPose.Result())
            self.executing.remove((handle, future))

    def spin(self, seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.01)

    def wait(self, predicate, timeout=6):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            self.executor.spin_once(timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError('Timed out waiting for external-navigation contract')

    def start(self):
        return self.coordinator.start(None, SimpleNamespace())

    def stop(self):
        return self.coordinator.stop_service_callback(None, SimpleNamespace())

    def close(self):
        try:
            if self.coordinator is not None:
                self.stop()
            self.mode = 'abort'
            self.spin(0.2)
        finally:
            if self.server is not None:
                self.server.destroy()
            self.executor.shutdown(timeout_sec=2)
            if self.coordinator is not None:
                self.coordinator.destroy_node()
            self.node.destroy_node()
            self.rclpy.shutdown(context=self.context)


@pytest.fixture
def world():
    world = World()
    try:
        yield world
    finally:
        world.close()


def test_preview_cannot_dispatch_even_with_available_server():
    world = World(dispatch=False)
    try:
        assert world.coordinator.client is None
        assert world.start().success
        world.spin(0.3)
        assert world.coordinator.current is not None
        assert not world.goals and world.coordinator.goals_sent == 0
        topics = dict(world.node.get_topic_names_and_types())
        assert not any('cmd_vel' in name or name.endswith('/simple_cmd') for name in topics)
        assert not world.coordinator.inflight
    finally:
        world.close()


def test_external_goal_contract_and_confirmed_cancellation(world):
    assert not world.goals  # startup is idle
    assert world.start().success
    center = world.coordinator.center
    assert world.start().success and world.coordinator.center == center
    world.wait(lambda: bool(world.goals))
    goal = world.goals[0]
    assert goal.pose.header.frame_id == 'map' and goal.behavior_tree == ''
    assert math.hypot(goal.pose.pose.position.x, goal.pose.pose.position.y) < 2.6
    q = goal.pose.pose.orientation
    assert q.z * q.z + q.w * q.w == pytest.approx(1)
    assert world.stop().success
    assert not world.start().success  # cannot overlap an unresolved goal
    world.wait(lambda: not world.coordinator.inflight)
    assert world.cancel_count == 1
    assert world.start().success


@pytest.mark.parametrize('failure', ['sensors', 'obstacle', 'stale_scan', 'lifecycle', 'map', 'deadline', 'radius'])
def test_health_and_mission_bounds_cancel_without_velocity_output(world, failure):
    assert world.start().success
    world.wait(lambda: bool(world.goals))
    if failure == 'sensors':
        world.sensors = False
    elif failure == 'obstacle':
        world.obstacle = True
    elif failure == 'stale_scan':
        world.stale_stamp = True
    elif failure == 'lifecycle':
        world.active = False
    elif failure == 'map':
        world.map_enabled = False
        world.coordinator.map_received -= 31
    elif failure == 'deadline':
        world.coordinator.started_at -= 61
    else:
        world.coordinator.center = (10.0, 0.0)
    world.wait(lambda: not world.coordinator.running)
    world.wait(lambda: not world.coordinator.inflight)
    assert world.cancel_count == 1
    assert not any('cmd_vel' in n for n, _ in world.node.get_topic_names_and_types())


def test_goal_timeout_and_rejected_cancel_block_restart(world):
    world.accept_cancel = False
    assert world.start().success
    world.wait(lambda: bool(world.goals))
    world.coordinator.sent_at -= 31
    world.wait(lambda: world.coordinator.cancel_failed)
    assert not world.coordinator.running
    assert not world.start().success
    world.mode = 'abort'
    world.wait(lambda: not world.coordinator.inflight)
    assert not world.start().success  # failure remains latched for review


@pytest.mark.parametrize('mode', ['reject', 'abort', 'success'])
def test_goal_results_and_budget(world, mode):
    if mode == 'reject':
        world.accept_goals = False
    else:
        world.mode = mode
    assert world.start().success
    if mode == 'success':
        world.coordinator.goals_sent = 20
        world.wait(lambda: not world.coordinator.running)
        assert world.coordinator.reason == 'goal_budget_complete'
    else:
        world.wait(lambda: not world.coordinator.running)
        assert world.coordinator.reason == 'repeated_goal_failure'
        assert world.coordinator.failures == 3


def test_configured_namespace_uses_existing_services_and_action():
    world = World(namespace='/university')
    try:
        assert world.start().success
        world.wait(lambda: bool(world.goals))
        assert world.coordinator.nav2_active()
    finally:
        world.close()


def test_late_acceptance_after_stop_is_canceled_and_never_resumes(world):
    from concurrent.futures import Future
    pending, result = Future(), Future()
    canceled = []

    def cancel():
        canceled.append(True)
        reply = Future()
        reply.set_result(SimpleNamespace(return_code=0, goals_canceling=[object()]))
        return reply

    handle = SimpleNamespace(accepted=True, get_result_async=lambda: result, cancel_goal_async=cancel)
    world.coordinator.client = SimpleNamespace(server_is_ready=lambda: True, send_goal_async=lambda _: pending)
    assert world.start().success
    assert world.coordinator.pending
    assert world.stop().success
    assert not world.start().success
    pending.set_result(handle)
    assert canceled == [True] and not world.coordinator.running
    assert not world.start().success  # cancellation acknowledgement isn't terminal result
    result.set_result(SimpleNamespace(status=5))  # STATUS_CANCELED
    assert not world.coordinator.inflight
    assert not world.coordinator.running


def test_missing_server_and_invalid_map_fail_closed(world):
    world.server.destroy()
    world.server = None
    world.wait(lambda: not world.coordinator.client.server_is_ready())
    assert not world.start().success
    world.coordinator.on_map(SimpleNamespace(header=SimpleNamespace(frame_id='wrong')))
    assert world.coordinator.grid is None


def test_cli_calls_only_mission_services(world):
    import subprocess
    root = Path(__file__).resolve().parents[1]
    executable = root / '.colcon/university_install/lite3_locomotion/lib/lite3_locomotion/autonomy_control'
    for command in ('start', 'stop'):
        process = subprocess.Popen([str(executable), command], stdout=subprocess.PIPE,
                                   stderr=subprocess.STDOUT, text=True)
        try:
            world.wait(lambda: process.poll() is not None)
            output = process.communicate()[0]
            assert process.returncode == 0, output
            assert '/lite3_locomotion/arm' not in output
        finally:
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
    world.wait(lambda: not world.coordinator.inflight)
    assert not world.coordinator.running


def test_installed_launch_starts_one_preview_node_without_navigation_servers():
    import json
    import signal
    import subprocess
    from std_msgs.msg import String
    world = World(dispatch=False)
    process = None
    try:
        world.executor.remove_node(world.coordinator)
        world.coordinator.destroy_node()
        world.coordinator = None
        status = []
        world.node.create_subscription(String, '/lite3_locomotion/exploration_status',
                                       lambda message: status.append(json.loads(message.data)), 10)
        process = subprocess.Popen(['ros2', 'launch', 'lite3_locomotion', 'autonomy.launch.py'],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        world.wait(lambda: bool(status), timeout=10)
        assert process.poll() is None
        assert not status[-1]['navigation_enabled'] and not status[-1]['running']
        assert not world.goals
        world.spin(0.2)
        names = world.node.get_node_names_and_namespaces()
        assert sorted(names) == [('lite3_exploration_coordinator', '/'), ('mock_university_navigation', '/')]
        assert not any('cmd_vel' in name for name, _ in world.node.get_topic_names_and_types())
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3)
        world.close()
