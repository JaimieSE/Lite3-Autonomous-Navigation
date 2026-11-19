"""Relay robot sensor topics straight from the robot's system domain to this PC's group domain.

Why: the robot-side domain bridge (lite3_domain_bridge_64 -> _8, university-run) throttles the
high-rate topics it relays, while the drivers' own writers in domain 64 reach this PC at full rate:

  2026-11-18  /livox/imu     ~190 Hz in 64 -> 12-15 Hz, ~1.3 s late in 8  (Fast-LIO odometry 1.4 s stale)
  2026-11-19  /joint_states  197 Hz in 64  -> 23 Hz in 8 (4.8 Hz at worst; preflight refused navigation)
              /imu/data      same pattern

  2026-11-19  /livox/lidar + /livox/points: median age 0.6 s in 64 -> 1.4 s in 8 (p95 2.3 s), 9.9 -> 4.8 Hz;
              the readiness check refused roaming (LiDAR odometry 1.0 s behind, sensing delay 1.28 s)

So subscribe in domain 64 and republish in domain 8 under our own names; navigation.launch.py remaps
the consumers (Fast-LIO: IMU + /livox/lidar; terrain_monitor: joints + body IMU + /livox/points; the
roam runner's raw-cloud check) to them. With nobody left subscribing to the bridge's cloud copies in
domain 8 the bridge stops sending them, so the robot->PC link carries each cloud once, as before.
The clouds (~400 KB, 10 Hz) get their own source context and thread so they never delay the 200 Hz
telemetry relays. The bridge's own topics
are left untouched for everyone else. Messages pass through serialized (no decode). Read-only on
the robot side: it only subscribes in domain 64.
"""

import struct
import threading
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu, JointState, PointCloud2

SOURCE_DOMAIN = 64
OUTPUT_DOMAIN = 8
# (message type, source topic in domain 64, output topic in domain 8, minimum healthy rate Hz, group)
RELAYS = (
    (Imu, '/livox/imu', '/lite3/livox_imu', 100.0, 'telemetry'),          # Fast-LIO
    (JointState, '/joint_states', '/lite3/joint_states', 100.0, 'telemetry'),  # terrain_monitor (body pose)
    (Imu, '/imu/data', '/lite3/imu_data', 100.0, 'telemetry'),            # terrain_monitor (body tilt)
    (CustomMsg, '/livox/lidar', '/lite3/livox_lidar', 8.0, 'clouds'),     # Fast-LIO
    (PointCloud2, '/livox/points', '/lite3/livox_points', 8.0, 'clouds'),  # terrain_monitor, roam check
)
DEPTH = {'telemetry': (50, 200), 'clouds': (2, 5)}      # (in, out) queue depths; clouds are ~400 KB each


def _node(domain, name):
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    node = Node(name, context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    return context, node, executor


def main():
    out_ctx, out_node, out_exec = _node(OUTPUT_DOMAIN, 'lite3_sensor_relay')
    src_ctx, src_node, src_exec = _node(SOURCE_DOMAIN, 'lite3_sensor_relay_src')
    cloud_ctx, cloud_node, cloud_exec = _node(SOURCE_DOMAIN, 'lite3_sensor_relay_clouds')
    sources = {'telemetry': src_node, 'clouds': cloud_node}
    # Best effort in: never back-pressure the drivers. Reliable out: the consumers are local and
    # Fast-LIO subscribes reliable (best-effort readers match a reliable writer too).
    counts = {}
    # Per 10 s window: arrival times, header age on arrival (wall clock - stamp; the robot computer and
    # this PC are NTP-synced) and publish duration, so the log shows whether data is already late or
    # gappy when it reaches the relay, or the relay itself delays it.
    windows = {}
    for msg_type, source, output, _, group in RELAYS:
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
            windows[source].append((arrived, age, time.monotonic() - arrived))

        sources[group].create_subscription(msg_type, source, relay, in_qos, raw=True)
        out_node.get_logger().info(f'relaying {source} (domain {SOURCE_DOMAIN}) -> {output} (domain {OUTPUT_DOMAIN})')
    threading.Thread(target=out_exec.spin, daemon=True).start()
    threading.Thread(target=cloud_exec.spin, daemon=True).start()

    def report():
        last, last_t = dict(counts), time.monotonic()
        while rclpy.ok(context=src_ctx):
            time.sleep(10.0)
            now = time.monotonic()
            parts, low = [], False
            timing = []
            for _, source, _, minimum, _ in RELAYS:
                rate = (counts[source] - last[source]) / (now - last_t)
                parts.append(f'{source} {rate:.0f} Hz')
                low |= rate < minimum
                window, windows[source] = windows[source], []
                if len(window) >= 2:
                    gap = max(b[0] - a[0] for a, b in zip(window, window[1:]))
                    ages = sorted(w[1] for w in window)
                    timing.append(f'{source} gap {gap:.2f} s age p50 {ages[len(ages) // 2]:.2f} '
                                  f'p95 {ages[int(0.95 * (len(ages) - 1))]:.2f} max {ages[-1]:.2f} s '
                                  f'publish max {max(w[2] for w in window) * 1000:.0f} ms')
            last, last_t = dict(counts), now
            log = out_node.get_logger()
            (log.warn if low else log.info)('relay ' + ', '.join(parts))
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
