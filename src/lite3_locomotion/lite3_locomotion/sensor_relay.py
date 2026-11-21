"""Relay robot sensor topics straight from the robot's system domain to this PC's group domain.

Why: the robot-side domain bridge (lite3_domain_bridge_64 -> _8, university-run) throttles the
high-rate topics it relays, while the drivers' own writers in domain 64 reach this PC at full rate:

  2026-11-18  /livox/imu     ~190 Hz in 64 -> 12-15 Hz, ~1.3 s late in 8  (Fast-LIO odometry 1.4 s stale)
  2026-11-19  /joint_states  197 Hz in 64  -> 23 Hz in 8 (4.8 Hz at worst; preflight refused navigation)
              /imu/data      same pattern

  2026-11-19  /livox/lidar + /livox/points: median age 0.6 s in 64 -> 1.4 s in 8 (p95 2.3 s), 9.9 -> 4.8 Hz;
              the readiness check refused roaming (LiDAR odometry 1.0 s behind, sensing delay 1.28 s)

Subscribe to the native Livox scan once. lite3_livox_cloud produces PointCloud2 locally.
Both consumers retain the original sensor frame and acquisition time. Cloud and telemetry
subscriptions use separate DDS participants and executor threads.
"""

import os
import struct
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2, PointField

SOURCE_DOMAIN = 64
OUTPUT_DOMAIN = 8
# (message type, source topic in domain 64, output topic in domain 8, minimum healthy rate Hz, group)
RELAYS = (
    (Imu, '/livox/imu', '/lite3/livox_imu', 100.0, 'telemetry'),          # Fast-LIO
    (JointState, '/joint_states', '/lite3/joint_states', 100.0, 'telemetry'),  # terrain_monitor (body pose)
    (Imu, '/imu/data', '/lite3/imu_data', 100.0, 'telemetry'),            # terrain_monitor (body tilt)
    (CustomMsg, '/livox/lidar', '/lite3/livox_lidar', 8.0, 'clouds'),     # Fast-LIO
)
DEPTH = {'telemetry': (50, 200), 'clouds': (2, 5)}      # (in, out) queue depths; clouds are ~400 KB each


def _node(domain, name):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    node = Node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return context, node, executor


DEPTH_SOURCE, DEPTH_INFO, DEPTH_OUTPUT = '/camera/depth/image_raw', '/camera/depth/camera_info', '/lite3/depth_points'


def _depth_relay(src, out, counts, windows, previous_arrival):
    """RealSense depth image (domain 64) -> sparse base_link points on /lite3/depth_points (domain 8).
    Own DDS participant so the 0.6 MB images never queue behind the telemetry or LiDAR. See depth_points.py."""
    import numpy as np
    from . import depth_points as D
    intrinsics = {}
    pub = out.create_publisher(PointCloud2, DEPTH_OUTPUT, QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE))
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    counts[DEPTH_SOURCE] = 0

    def on_info(msg):
        intrinsics['k'] = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(msg):
        if 'k' not in intrinsics or msg.encoding not in ('16UC1', 'mono16'):
            return
        arrived = time.monotonic()
        depth = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        pts = D.depth_to_base(depth, *intrinsics['k'])
        cloud = PointCloud2()
        cloud.header.stamp = msg.header.stamp
        cloud.header.frame_id = 'base_link'
        cloud.height, cloud.width = 1, len(pts)
        cloud.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                        for i, n in enumerate('xyz')]
        cloud.point_step, cloud.row_step, cloud.is_dense = 12, 12 * len(pts), True
        cloud.data = pts.astype('<f4').tobytes()
        pub.publish(cloud)
        counts[DEPTH_SOURCE] += 1
        previous_arrival[DEPTH_SOURCE] = arrived

    src.create_subscription(CameraInfo, DEPTH_INFO, on_info, qos)
    src.create_subscription(Image, DEPTH_SOURCE, on_depth, qos)
    out.get_logger().info(f'depth camera: {DEPTH_SOURCE} (domain {SOURCE_DOMAIN}) -> {DEPTH_OUTPUT} (domain {OUTPUT_DOMAIN})')


def main():
    out_ctx, out_node, out_exec = _node(OUTPUT_DOMAIN, 'lite3_sensor_relay')
    src_ctx, src_node, src_exec = _node(SOURCE_DOMAIN, 'lite3_sensor_relay_src')
    cloud_ctx, cloud_node, cloud_exec = _node(SOURCE_DOMAIN, 'lite3_sensor_relay_clouds')
    # In system mode FAST-LIO reads the Livox IMU on .64 itself.
    relays = [r for r in RELAYS if r[1] != '/livox/imu' or os.environ.get('LITE3_LIO_SOURCE', 'system') == 'local']
    sources = {'telemetry': src_node, 'clouds': cloud_node}
    # Best effort in: never back-pressure the drivers. Reliable out: the consumers are local and
    # Fast-LIO subscribes reliable (best-effort readers match a reliable writer too).
    counts = {}
    # Per 10 s window: arrival times, header age on arrival (wall clock - stamp; the robot computer and
    # this PC are NTP-synced) and publish duration, so the log shows whether data is already late or
    # gappy when it reaches the relay, or the relay itself delays it.
    windows = {}
    previous_arrival = {}
    for msg_type, source, output, _, group in relays:
        depth_in, depth_out = DEPTH[group]
        out_qos = QoSProfile(depth=depth_out, reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST)
        in_qos = QoSProfile(depth=depth_in, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        pub = out_node.create_publisher(msg_type, output, out_qos)
        counts[source] = 0
        windows[source] = []

        def relay(raw, pub=pub, source=source):
            arrived = time.monotonic()
            pub.publish(raw)
            counts[source] += 1
            try:
                sec, nsec = struct.unpack_from('<iI', raw, 4)     # CDR header.stamp after the 4-byte encapsulation
                age = time.time() - (sec + nsec * 1e-9)
            except struct.error:
                age = float('nan')
            gap = arrived - previous_arrival.get(source, arrived)
            previous_arrival[source] = arrived
            windows[source].append((gap, age, time.monotonic() - arrived))

        sources[group].create_subscription(msg_type, source, relay, in_qos, raw=True)
        out_node.get_logger().info(f'relaying {source} (domain {SOURCE_DOMAIN}) -> {output} (domain {OUTPUT_DOMAIN})')
    if os.environ.get('LITE3_LIO_SOURCE', 'system') == 'system':
        # Expose the bridged scan only in system mode so local FAST-LIO can own /scan.
        scan_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        scan_pub = out_node.create_publisher(LaserScan, '/scan', scan_qos)
        out_node.create_subscription(
            LaserScan, '/group8_lio/scan', scan_pub.publish, scan_qos, raw=True)
    if os.environ.get('LITE3_DEPTH_CAMERA', '1') == '1':
        depth_ctx, depth_node, depth_exec = _node(SOURCE_DOMAIN, 'lite3_sensor_relay_depth')
        _depth_relay(depth_node, out_node, counts, windows, previous_arrival)
        threading.Thread(target=depth_exec.spin, daemon=True).start()
    threading.Thread(target=out_exec.spin, daemon=True).start()
    threading.Thread(target=cloud_exec.spin, daemon=True).start()

    def report():
        last, last_t = dict(counts), time.monotonic()
        while rclpy.ok(context=src_ctx):
            time.sleep(10.0)
            now = time.monotonic()
            parts, low = [], False
            timing = []
            if DEPTH_SOURCE in counts:
                parts.append(f'{DEPTH_SOURCE} {(counts[DEPTH_SOURCE] - last.get(DEPTH_SOURCE, 0)) / (now - last_t):.0f} Hz')
            for _, source, _, minimum, _ in relays:
                rate = (counts[source] - last[source]) / (now - last_t)
                parts.append(f'{source} {rate:.0f} Hz')
                low |= rate < minimum
                window, windows[source] = windows[source], []
                if not window:
                    last_seen = previous_arrival.get(source)
                    timing.append(f'{source} silent {now - last_seen:.2f} s' if last_seen else f'{source} waiting')
                    continue
                gap = max(w[0] for w in window)
                detail = f'{source} gap {gap:.2f} s'
                # Body telemetry headers on this robot use a different clock from group8.
                if source.startswith('/livox/'):
                    ages = sorted(w[1] for w in window)
                    detail += (f' age p50 {ages[len(ages) // 2]:.2f} '
                               f'p95 {ages[int(0.95 * (len(ages) - 1))]:.2f} max {ages[-1]:.2f} s')
                timing.append(detail + f' publish max {max(w[2] for w in window) * 1000:.0f} ms')
            last, last_t = dict(counts), now
            log = out_node.get_logger()
            message = 'relay ' + ', '.join(parts)
            if low:
                log.warning(message)
            else:
                log.info(message)
            log.info('relay timing ' + ' | '.join(timing))
    threading.Thread(target=report, daemon=True).start()
    try:
        src_exec.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        for ctx, node, ex in ((src_ctx, src_node, src_exec), (cloud_ctx, cloud_node, cloud_exec),
                              (out_ctx, out_node, out_exec)):
            ex.shutdown(timeout_sec=1.0)
            node.destroy_node()
            if rclpy.ok(context=ctx):
                rclpy.shutdown(context=ctx)


if __name__ == '__main__':
    main()
