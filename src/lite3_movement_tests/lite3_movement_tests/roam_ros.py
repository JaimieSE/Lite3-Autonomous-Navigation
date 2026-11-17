"""ROS adapters for M10: the existing NavRobot plus the SLAM map, the Nav2 global costmap and the
raw LiDAR point clouds. Subscriptions only; nothing here publishes beyond NavRobot itself."""

import math
import threading
import time

from .ros_io import NAV_WIRING, NavRobot


def grid_from_occupancy(msg):
    from lite3_locomotion.frontiers import Grid
    q = msg.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return Grid(int(msg.info.width), int(msg.info.height), float(msg.info.resolution),
                [int(v) for v in msg.data], float(msg.info.origin.position.x),
                float(msg.info.origin.position.y), yaw)


class RoamRobot(NavRobot):
    """NavRobot + /map (slam_toolbox), /global_costmap/costmap (incl. the terrain point-cloud
    hazards), /livox/points (raw LiDAR: rate and serialized size) and /cloud_registered_body
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
        from sensor_msgs.msg import PointCloud2
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
        node.create_subscription(PointCloud2, '/livox/points', lambda raw: t.raw_cloud.add(
            time.monotonic(), len(raw)), qos_profile_sensor_data, raw=True)
        node.create_subscription(PointCloud2, '/cloud_registered_body', lambda raw: t.lio_cloud.add(
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

    def map_grid(self):
        return self._grid('map')

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
