"""ROS adapters for M10: the existing NavRobot plus the SLAM map, the Nav2 global costmap and the
raw LiDAR point clouds. Subscriptions only; nothing here publishes beyond NavRobot itself."""

import os
import math
import threading
import time

from .ros_io import NAV_WIRING, NavRobot

SCENE_TOPIC = '/scene/detections'


def grid_from_occupancy(msg):
    from lite3_locomotion.frontiers import Grid
    q = msg.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Grid(int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                [int(v) for v in msg.data], float(msg.info.origin.position.x),
                float(msg.info.origin.position.y), yaw)


class RoamRobot(NavRobot):
    """NavRobot + /map (slam_toolbox), /global_costmap/costmap (incl. the terrain point-cloud
    hazards), /lite3/livox_points (raw LiDAR via sensor_relay: rate and serialized size) and /cloud_registered_body
    (Fast-LIO alive)."""

    def _extra_setup(self):
        super()._extra_setup()
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan, PointCloud2
        # Large messages get their OWN node, context and executor thread, so they can never starve
        # the joint/IMU/odometry callbacks the safety checks depend on. The point clouds are taken
        # RAW (serialized, never deserialized): liveness needs only their rate and size.
        context = Context()
        rclpy.init(args=[], context=context, domain_id=self.wiring.velocity_domain,
                   signal_handler_options=SignalHandlerOptions.NO)
        node = Node(f'{self.node_name}_maps', context=context, use_global_arguments=False,
                    start_parameter_services=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        self._contexts.append((context, node, executor,
                               threading.Thread(target=self._spin, args=(executor,), daemon=True)))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._grids = {'map': None, 'costmap': None}
        self._grid_cache = {}
        self._grid_lock = threading.Lock()
        t = self.telemetry

        def keep(name):
            def callback(msg):
                with self._grid_lock:
                    self._grids[name] = (time.monotonic(), msg)
            return callback

        node.create_subscription(OccupancyGrid, '/map', keep('map'), latched)
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', keep('costmap'), latched)
        # terrain_monitor's level grid around the robot: observed floor counts as seen free for walks
        node.create_subscription(OccupancyGrid, '/lite3_terrain/grid', keep('terrain'), 1)
        # Scene understanding (a teammate's node, ~/scene_understanding/scene_node.py): JSON String with
        # objects[{label, confidence, xyxy}] and optionally 'qr'. Passive: roam only logs and keeps them.
        from std_msgs.msg import String
        self.scene = []
        node.create_subscription(String, SCENE_TOPIC, lambda m: self.scene.append((time.monotonic(), m.data)), 10)
        # sensor_relay's domain-64 copy: subscribing the bridged /livox/points would make the bridge
        # send every cloud again over the robot link (and it arrives ~0.8 s later)
        node.create_subscription(PointCloud2, '/lite3/livox_points', lambda raw: t.raw_cloud.add(
            time.monotonic(), len(raw)), qos_profile_sensor_data, raw=True)
        if os.environ.get('LITE3_LIO_SOURCE', 'system') == 'system':
            lio_type, lio_topic = LaserScan, '/scan'
        else:
            lio_type, lio_topic = PointCloud2, '/cloud_registered_body'
        node.create_subscription(lio_type, lio_topic, lambda raw: t.lio_cloud.add(
            time.monotonic(), len(raw)), qos_profile_sensor_data, raw=True)

    def _grid(self, name):
        with self._grid_lock:
            item = self._grids.get(name)
        if item is None:
            return None
        stamp, msg = item
        cached = self._grid_cache.get(name)
        if cached is None or cached[0] != stamp:          # convert each new message once
            cached = (stamp, grid_from_occupancy(msg))
            self._grid_cache[name] = cached
        return cached

    def set_terrain_proximity_stop(self, enabled, timeout_s=2.0):
        """terrain_monitor's proximity_stop parameter (stairs: off while climbing, back on after)."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        node = self._nodes[self.wiring.velocity_domain]
        client = node.create_client(SetParameters, '/lite3_terrain_monitor/set_parameters')
        try:
            if not client.wait_for_service(timeout_sec=timeout_s):
                return False
            value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=bool(enabled))
            future = client.call_async(SetParameters.Request(parameters=[Parameter(name='proximity_stop', value=value)]))
            deadline = time.monotonic() + timeout_s
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            return future.done() and all(r.successful for r in future.result().results)
        finally:
            node.destroy_client(client)

    def map_grid(self):
        return self._grid('map')

    def terrain_floor(self):
        """(monotonic arrival, {'ox','oy','res','w','h','data'}) of the latest terrain grid, or None."""
        with self._grid_lock:
            item = self._grids.get('terrain')
        if item is None:
            return None
        stamp, msg = item
        cached = self._grid_cache.get('terrain')
        if cached is None or cached[0] != stamp:
            cached = (stamp, {'ox': msg.info.origin.position.x, 'oy': msg.info.origin.position.y,
                              'res': msg.info.resolution, 'w': msg.info.width, 'h': msg.info.height,
                              'data': bytes(b & 0xFF for b in msg.data)})
            self._grid_cache['terrain'] = cached
        return cached

    def costmap_grid(self):
        return self._grid('costmap')


class WatchdogRobot(RoamRobot):
    """Read-only instance for the independent watchdog process (its own node name)."""
    node_name = 'lite3_roam_watchdog'

    def __init__(self, wiring=NAV_WIRING, cancel_flag=None):
        super().__init__(wiring, allow_commands=False, cancel_flag=cancel_flag)


class DryRoamRobot(RoamRobot):
    """Read-only dry roaming: its own node name, so it never looks like the test runner."""
    node_name = 'lite3_roam_dry'
