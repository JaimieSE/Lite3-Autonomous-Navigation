"""Visualize frontier suggestions only. No Nav2 client or movement publisher."""

import argparse
import math
import sys
import time

from lite3_locomotion.frontiers import ExplorationResult, FrontierConfig, Grid, select_frontiers


MAP_TOPIC = '/map'
MARKER_TOPIC = '/lite3_locomotion/exploration_markers'
MAP_FRAME = 'map'
BASE_FRAME = 'base_link'


def parse_config(args):
    parser = argparse.ArgumentParser(description='Frontier visualization only; no robot commands',
                                     allow_abbrev=False)
    parser.add_argument('--radius', type=float, default=5.0)
    parser.add_argument('--clearance', type=float, default=0.30)
    options = parser.parse_args(args)
    try:
        return FrontierConfig(**vars(options))
    except ValueError as error:
        parser.error(str(error))


def grid_from_message(message):
    if message.header.frame_id != MAP_FRAME:
        raise ValueError('Only the fixed map frame is supported')
    q = message.info.origin.orientation
    components = (q.x, q.y, q.z, q.w)
    if (not all(math.isfinite(v) for v in components) or abs(q.x) > 1e-4 or
            abs(q.y) > 1e-4 or abs(sum(v * v for v in components) - 1) > 1e-3):
        raise ValueError('Map origin must have a normalized planar quaternion')
    if message.info.width * message.info.height > 1_000_000:
        raise ValueError('Map exceeds preview cell budget')
    return Grid(message.info.width, message.info.height, message.info.resolution,
                tuple(message.data), message.info.origin.position.x,
                message.info.origin.position.y, 2 * math.atan2(q.z, q.w))


def create_preview_node(config, context):
    from geometry_msgs.msg import Point
    from nav_msgs.msg import OccupancyGrid
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    class ExplorationPreview(Node):
        def __init__(self):
            super().__init__('lite3_exploration_preview', namespace='/', context=context,
                             use_global_arguments=False, enable_rosout=False,
                             start_parameter_services=False)
            if (self.resolve_topic_name(MAP_TOPIC) != MAP_TOPIC or
                    self.resolve_topic_name(MARKER_TOPIC) != MARKER_TOPIC):
                self.destroy_node()
                raise RuntimeError('Preview topic isolation check failed')
            self.grid = None
            self.received_at = None
            self.map_error = 'waiting_for_map'
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.publisher = self.create_publisher(MarkerArray, MARKER_TOPIC, qos)
            self.subscription = self.create_subscription(OccupancyGrid, MAP_TOPIC, self.on_map, qos)
            self.timer = self.create_timer(2.0, self.update)
            self.get_logger().info(f'PREVIEW ONLY: {MAP_TOPIC} -> {MARKER_TOPIC}; no goals or velocity sent')

        def on_map(self, message):
            try:
                self.grid = grid_from_message(message)
                self.received_at = time.monotonic()
                self.map_error = ''
            except (ValueError, TypeError, OverflowError) as error:
                self.grid = None
                self.map_error = f'invalid_map: {error}'

        def marker(self, identifier, kind):
            marker = Marker()
            marker.header.frame_id = MAP_FRAME
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'lite3_exploration_preview'
            marker.id = identifier
            marker.type = kind
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.color.a = 1.0
            marker.lifetime = Duration(seconds=5.0).to_msg()
            return marker

        def publish(self, result, robot=None):
            messages = []
            clear = self.marker(0, Marker.SPHERE)
            clear.action = Marker.DELETEALL
            messages.append(clear)
            text = self.marker(1, Marker.TEXT_VIEW_FACING)
            text.pose.position.z = 0.8
            if robot is not None:
                text.pose.position.x, text.pose.position.y = robot
            text.scale.z = 0.18
            text.color.r, text.color.g, text.color.b = 1.0, 1.0, 1.0
            text.text = (f'PREVIEW ONLY — {result.status}\n'
                         f'{len(result.candidates)} candidates; {result.visited_cells} cells searched')
            messages.append(text)
            if robot is not None:
                boundary = self.marker(2, Marker.LINE_STRIP)
                boundary.scale.x = 0.03
                boundary.color.r, boundary.color.g, boundary.color.b = 0.3, 0.6, 1.0
                for step in range(65):
                    angle = step * 2 * math.pi / 64
                    boundary.points.append(Point(x=robot[0] + config.radius * math.cos(angle),
                                                 y=robot[1] + config.radius * math.sin(angle), z=0.05))
                messages.append(boundary)
            points = self.marker(3, Marker.SPHERE_LIST)
            points.scale.x = points.scale.y = points.scale.z = 0.16
            points.color.r, points.color.g, points.color.b = 1.0, 0.75, 0.0
            for candidate in result.candidates:
                points.points.append(Point(x=candidate.x, y=candidate.y, z=0.12))
            messages.append(points)
            if result.selected is not None:
                selected = result.selected
                arrow = self.marker(4, Marker.ARROW)
                arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = selected.x, selected.y, 0.2
                arrow.pose.orientation.z = math.sin(selected.yaw / 2)
                arrow.pose.orientation.w = math.cos(selected.yaw / 2)
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.6, 0.12, 0.12
                arrow.color.r, arrow.color.g, arrow.color.b = 0.0, 1.0, 0.2
                messages.append(arrow)
            output = MarkerArray()
            output.markers = messages
            self.publisher.publish(output)
            self.get_logger().info(text.text.replace('\n', '; '), throttle_duration_sec=10.0)

        def update(self):
            if self.grid is None:
                self.publish(ExplorationResult(self.map_error))
                return
            if time.monotonic() - self.received_at > 30:
                self.publish(ExplorationResult('map_receipt_stale'))
                return
            try:
                transform = self.buffer.lookup_transform(MAP_FRAME, BASE_FRAME, Time())
                age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                robot = transform.transform.translation
                if not -0.5 <= age <= 2.0 or not all(math.isfinite(v) for v in (robot.x, robot.y)):
                    self.publish(ExplorationResult('robot_transform_stale_or_invalid'))
                    return
            except TransformException:
                self.publish(ExplorationResult('waiting_for_map_to_base_link_tf'))
                return
            result = select_frontiers(self.grid, robot.x, robot.y, config)
            self.publish(result, (robot.x, robot.y))

    return ExplorationPreview()


def main(args=None):
    config = parse_config(sys.argv[1:] if args is None else args)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context)
        node = create_preview_node(config, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if executor is not None:
                executor.shutdown()
        finally:
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
