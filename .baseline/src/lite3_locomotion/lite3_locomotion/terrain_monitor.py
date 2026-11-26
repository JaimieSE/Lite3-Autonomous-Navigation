"""terrain_monitor: perception-only ROS node over the existing Lite3 sensors.

Inputs (all existing): /livox/points (Mid-360), /imu/data and /joint_states
(vendor motion_receiver via the domain bridge), TF base_link -> livox_frame
(driver static TF, bridged), optional TF odom -> base_link (university Fast-LIO
odom_relay) for accumulation of recent scans.

Outputs (new, read-only consumers): /lite3_terrain/grid (OccupancyGrid),
/lite3_terrain/hazards (PointCloud2 of obstacle/drop cells in base_link, suitable
as an extra Nav2 costmap observation source), /lite3_terrain/status (JSON), /lite3_terrain/stop (Bool) for a twist_mux lock:
true when an obstacle/drop is within the proximity zone or inputs are invalid;
held true for 1 s after clearing. If this node stops, the lock's timeout engages.
ROS parameters stop_ahead_m (from base_link centre) / body_margin_m (round the footprint) may be raised at
runtime for a test, never below the validated defaults (terrain.STOP_LIMITS); the
active values are reported in status.stop.
The stop signal is a perception output; blocking motion is the mux's job.
It has no velocity, posture, mode or service interfaces.
"""

import argparse
import json
import math
import time

import numpy as np

from . import terrain as T

THIGH_M, SHANK_M = 0.200, 0.21012   # Lite3 URDF
KNEES = (2, 5, 8, 11)


def leg_floor_reference(positions):
    """Floor height in the level frame from the four knee angles (hips lie at base_link z)."""
    if len(positions) != 12 or not all(math.isfinite(p) for p in positions):
        return None
    ext = [math.sqrt(THIGH_M ** 2 + SHANK_M ** 2 + 2 * THIGH_M * SHANK_M * math.cos(abs(positions[k])))
           for k in KNEES]
    return -float(np.mean(ext))


def decode_xyz(msg):
    offsets = {f.name: f.offset for f in msg.fields}
    if not {'x', 'y', 'z'} <= set(offsets) or any(
            f.datatype != 7 for f in msg.fields if f.name in 'xyz'):   # FLOAT32
        raise ValueError('cloud lacks float32 x/y/z')
    raw = np.frombuffer(bytes(msg.data), np.uint8)
    count = msg.width * msg.height
    raw = raw[:count * msg.point_step].reshape(count, msg.point_step)
    xyz = np.stack([raw[:, offsets[k]:offsets[k] + 4].copy().view('<f4')[:, 0] for k in 'xyz'], 1)
    xyz = xyz.astype(np.float64)
    return xyz[np.isfinite(xyz).all(1) & (np.linalg.norm(xyz, axis=1) > 0.05)]


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-accumulate', action='store_true')
    parser.add_argument('--max-step', type=float, default=T.TerrainParams.max_step_m)
    parser.add_argument('--max-drop', type=float, default=T.TerrainParams.max_drop_m)
    options, _ = parser.parse_known_args(args)
    params = T.TerrainParams(max_step_m=options.max_step, max_drop_m=options.max_drop)

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import Imu, JointState, PointCloud2, PointField
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    rclpy.init(args=args)
    node = Node('lite3_terrain_monitor')
    buffer = Buffer()
    TransformListener(buffer, node)
    state = {'tilt': None, 'legs': None, 'busy': False, 'extrinsic': None}
    accumulator = None if options.no_accumulate else T.Accumulator()
    grid_pub = node.create_publisher(OccupancyGrid, '/lite3_terrain/grid', 1)
    hazard_pub = node.create_publisher(PointCloud2, '/lite3_terrain/hazards', 1)
    status_pub = node.create_publisher(String, '/lite3_terrain/status', 1)
    stop_pub = node.create_publisher(Bool, '/lite3_terrain/stop', 1)
    from rcl_interfaces.msg import SetParametersResult
    state['stop_params'] = T.StopParams()
    state['last_hazard'] = -math.inf
    for name in T.STOP_LIMITS:
        node.declare_parameter(name, getattr(state['stop_params'], name))
    # Operator 2026-11-20 (docs-only stops): false = obstacles go to the Nav2 costmaps only, as in the
    # vendor navigation; the lock then only engages on invalid/stale inputs (or if this node dies).
    state['proximity_stop'] = bool(node.declare_parameter('proximity_stop', True).value)

    def on_parameters(parameters):
        """Proximity-stop distances: may be raised for a test, never below the validated default."""
        params = state['stop_params']
        try:
            for p in parameters:
                if p.name in T.STOP_LIMITS:
                    params = T.updated_stop_params(params, p.name, p.value)
        except ValueError as error:
            return SetParametersResult(successful=False, reason=str(error))
        state['stop_params'] = params
        node.get_logger().info(f'proximity stop now ahead {params.stop_ahead_m} m, '
                               f'body margin {params.body_margin_m} m')
        return SetParametersResult(successful=True)
    node.add_on_set_parameters_callback(on_parameters)

    def on_imu(msg):
        q = msg.orientation
        r = T.quat_to_matrix(q.x, q.y, q.z, q.w)
        state['tilt'] = (math.atan2(r[2, 1], r[2, 2]), -math.asin(max(-1.0, min(1.0, r[2, 0]))),
                         time.monotonic())

    def on_joints(msg):
        state['legs'] = (leg_floor_reference(list(msg.position)), time.monotonic())

    def transform(target, source):
        tf = buffer.lookup_transform(target, source, Time())
        q, t = tf.transform.rotation, tf.transform.translation
        return T.quat_to_matrix(q.x, q.y, q.z, q.w), np.array([t.x, t.y, t.z])

    def on_cloud(msg):
        if state['busy']:
            return
        state['busy'] = True
        started = time.monotonic()
        status = {'stamp': f'{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}',
                  'perception_only': True, 'problems': [],
                  # cloud age when this callback starts (wall clock - stamp): delay before processing
                  'input_age_s': round(time.time() - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9), 3)}
        hazard = (True, 'not evaluated')
        try:
            now = time.monotonic()
            if state['extrinsic'] is None:
                try:
                    state['extrinsic'] = transform('base_link', msg.header.frame_id)
                except Exception as error:  # noqa: BLE001
                    status['problems'].append(f'no base_link->{msg.header.frame_id} TF: {error}')
            tilt, legs = state['tilt'], state['legs']
            if tilt is None or now - tilt[2] > 0.5:
                status['problems'].append('IMU attitude stale')
            if legs is None or legs[0] is None or now - legs[1] > 0.5:
                status['problems'].append('joint telemetry stale or invalid')
            if status['problems']:
                return
            r, t = state['extrinsic']
            base, _ = T.to_level(decode_xyz(msg), r, t, 0.0, 0.0)
            base = base[T.self_filter(base, params)]
            status['points'] = int(len(base))
            status['accumulated'] = False
            if accumulator is not None:
                try:
                    odom_r, odom_t = transform('odom', 'base_link')
                    accumulator.add(now, base, odom_r, odom_t)
                    base = accumulator.base_points(odom_r, odom_t)
                    base = base[T.self_filter(base, params)]
                    status['accumulated'] = True
                    status['accumulated_points'] = int(len(base))
                except Exception:  # noqa: BLE001 - university odometry not running
                    pass
            level_r = T.level_rotation(tilt[0], tilt[1])
            level = base @ level_r.T
            floor_z, source = T.estimate_floor(level, legs[0], params)
            grid = T.build_grid(level, floor_z, params, source)
            hazard = T.proximity_stop(grid, state['stop_params'])
            status.update({
                'floor_z_m': round(floor_z, 3), 'floor_source': source,
                'leg_floor_reference_m': round(legs[0], 3),
                'tilt_rad': [round(tilt[0], 4), round(tilt[1], 4)],
                'labels': T.label_counts(grid), 'corridor': T.corridor_summary(grid, params),
                'params': {'max_step_m': params.max_step_m, 'max_drop_m': params.max_drop_m,
                           'validated_limits': False},
            })
            occ = OccupancyGrid()
            occ.header.stamp = msg.header.stamp
            occ.header.frame_id = 'base_link'   # level-frame grid; tilt is small when walking
            occ.info.resolution = float(grid.resolution)
            occ.info.width = grid.labels.shape[1]
            occ.info.height = grid.labels.shape[0]
            occ.info.origin.position.x, occ.info.origin.position.y = grid.origin
            occ.info.origin.position.z = float(floor_z)
            occ.info.origin.orientation.w = 1.0
            occ.data = T.occupancy_values(grid).ravel().tolist()
            grid_pub.publish(occ)
            hazards = T.hazard_points(grid) @ level_r   # level -> base_link
            cloud = PointCloud2()
            cloud.header = occ.header
            cloud.height, cloud.width = 1, len(hazards)
            cloud.fields = [PointField(name=n, offset=4 * i, datatype=7, count=1)
                            for i, n in enumerate('xyz')]
            cloud.point_step, cloud.row_step = 12, 12 * len(hazards)
            cloud.is_dense = True
            cloud.data = hazards.astype('<f4').tobytes()
            hazard_pub.publish(cloud)
        except Exception as error:  # noqa: BLE001 - report, never crash the monitor
            status['problems'].append(f'{type(error).__name__}: {error}')
        finally:
            if not node.context.ok():   # shutting down: nothing may be published any more
                state['busy'] = False
                return
            if not state['proximity_stop']:
                hazard = (False, None)          # obstacles: costmaps only (docs-only stops)
            if status['problems']:
                hazard = (True, 'inputs invalid: ' + '; '.join(status['problems']))
            now = time.monotonic()
            if hazard[0]:
                state['last_hazard'] = now
            stop_params = state['stop_params']
            stop = now - state['last_hazard'] < stop_params.release_hold_s
            status['stop'] = {'active': stop, 'reason': hazard[1] if hazard[0] else
                              ('release hold' if stop else None),
                              'ahead_m': stop_params.stop_ahead_m,
                              'body_margin_m': stop_params.body_margin_m,
                              'proximity_stop': state['proximity_stop']}
            stop_pub.publish(Bool(data=stop))
            status['processing_s'] = round(time.monotonic() - started, 3)
            status_pub.publish(String(data=json.dumps(status)))
            state['busy'] = False

    node.create_subscription(Imu, '/imu/data', on_imu, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', on_joints, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/livox/points', on_cloud, qos_profile_sensor_data)
    from rclpy.executors import ExternalShutdownException
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:   # noqa: BLE001 - rclpy raises RCLError if shutdown races the executor
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
