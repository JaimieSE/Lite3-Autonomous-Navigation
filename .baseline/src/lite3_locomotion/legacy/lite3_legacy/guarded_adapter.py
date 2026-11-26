"""Separate Nav2 command adapter. Default output is dry-run; starts disarmed.

Robot output uses only /lite3_robot_link/cmd_vel, never the original test topic.
No SDK mode, Manual-mode, posture, or emergency-stop integration is included.
"""

import argparse
import json
import math
import signal
import sys
import time

from lite3_locomotion.command_policy import Velocity
from lite3_legacy.movement_guard import MovementGuard, scan_is_clear


NAV_COMMAND = '/lite3_autonomy/nav_cmd_vel'
DRY_OUTPUT = '/lite3_locomotion/guarded_test_cmd_vel'
ROBOT_OUTPUT = '/lite3_robot_link/cmd_vel'
ENABLED = '/lite3_locomotion/autonomy_enabled'
STATUS = '/lite3_locomotion/guard_status'
ARM = '/lite3_locomotion/arm'
STOP = '/lite3_locomotion/stop'
BRIDGE_READY = '/lite3_robot_link/ready'
BRIDGE_AUTHORIZE = '/lite3_robot_link/session_ack'


def parse_options(args):
    parser = argparse.ArgumentParser(description='Disarmed Nav2 command gate; dry-run by default', allow_abbrev=False)
    parser.add_argument('--output-mode', choices=['dry-run', 'robot'], default='dry-run')
    parser.add_argument('--session-seconds', type=float, default=60.0)
    parser.add_argument('--radius', type=float, default=3.0)
    options = parser.parse_args(args)
    if not math.isfinite(options.radius) or not 0 < options.radius <= 10:
        parser.error('radius must be in (0, 10] metres')
    try:
        MovementGuard(session_seconds=options.session_seconds)
    except ValueError as error:
        parser.error(str(error))
    return options


def create_node(options, context):
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformException, TransformListener

    class GuardedAdapter(Node):
        def __init__(self):
            super().__init__('lite3_guarded_adapter', namespace='/', context=context,
                             use_global_arguments=False, enable_rosout=False, start_parameter_services=False)
            self.output_topic = ROBOT_OUTPUT if options.output_mode == 'robot' else DRY_OUTPUT
            for name in (NAV_COMMAND, self.output_topic, ENABLED, STATUS, '/scan'):
                if self.resolve_topic_name(name) != name:
                    self.destroy_node()
                    raise RuntimeError('Adapter topic isolation failed')
            for name in (ARM, STOP):
                if self.resolve_service_name(name) != name:
                    self.destroy_node()
                    raise RuntimeError('Adapter service isolation failed')
            self.guard = MovementGuard(session_seconds=options.session_seconds)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.scan_received = None
            self.scan_stamp = None
            self.scan_clear = False
            self.center = None
            self.pose = None
            self.health_reason = 'waiting_for_scan_and_tf'
            self.bridge_ready = False
            self.bridge_received = None
            self.bridge_session = None
            self.publisher = self.create_publisher(Twist, self.output_topic, 1)
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.enabled_pub = self.create_publisher(Bool, ENABLED, status_qos)
            self.status_pub = self.create_publisher(String, STATUS, status_qos)
            self.command_sub = self.create_subscription(Twist, NAV_COMMAND, self.command, 1)
            self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
            if options.output_mode == 'robot':
                if (self.resolve_topic_name(BRIDGE_READY) != BRIDGE_READY or
                        self.resolve_topic_name(BRIDGE_AUTHORIZE) != BRIDGE_AUTHORIZE):
                    raise RuntimeError('Bridge handshake topic isolation failed')
                self.bridge_sub = self.create_subscription(String, BRIDGE_READY, self.bridge_status, 1)
                self.bridge_ack_pub = self.create_publisher(String, BRIDGE_AUTHORIZE, 1)
            self.arm_service = self.create_service(Trigger, ARM, self.arm)
            self.stop_service = self.create_service(Trigger, STOP, self.stop)
            self.timer = self.create_timer(0.05, self.tick)
            self.get_logger().info(f'DISARMED: output mode={options.output_mode}, topic={self.output_topic}')

        def bridge_status(self, message):
            try:
                state = json.loads(message.data)
                session = state['session_id']
                ready = state['enabled'] is True and isinstance(session, str) and len(session) == 32
            except (ValueError, KeyError, TypeError):
                ready, session = False, None
            if session != self.bridge_session:
                self.guard.stop('sender_session_changed')
            self.bridge_session = session
            self.bridge_ready = ready
            self.bridge_received = time.monotonic()

        def scan(self, message):
            self.scan_stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now() - Time.from_msg(message.header.stamp)).nanoseconds / 1e9
            coverage = abs(message.angle_increment) * max(0, len(message.ranges) - 1)
            self.scan_clear = (message.header.frame_id == 'base_link' and -0.2 <= age <= 0.5 and
                               math.isfinite(coverage) and 6.0 <= coverage <= 6.4 and
                               scan_is_clear(message.ranges, message.range_min, message.range_max))
            self.scan_received = time.monotonic()

        def healthy(self):
            scan_age = ((self.get_clock().now() - self.scan_stamp).nanoseconds / 1e9
                        if self.scan_stamp is not None else float('inf'))
            if (self.scan_received is None or time.monotonic() - self.scan_received > 0.5 or
                    not -0.2 <= scan_age <= 0.5 or not self.scan_clear):
                self.health_reason = 'scan_stale_invalid_or_obstacle_within_0.40m'
                return False
            try:
                transform = self.buffer.lookup_transform('map', 'base_link', Time())
                age = (self.get_clock().now() - Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                p = transform.transform.translation
                if not -0.2 <= age <= 0.5 or not all(math.isfinite(v) for v in (p.x, p.y)):
                    self.health_reason = 'robot_tf_stale_or_invalid'
                    return False
                self.pose = (p.x, p.y)
                if self.guard.armed and self.center is not None and math.dist(self.pose, self.center) >= options.radius:
                    self.health_reason = 'session_radius_exceeded'
                    return False
            except TransformException:
                self.health_reason = 'robot_tf_unavailable'
                return False
            if options.output_mode == 'robot':
                if (not self.bridge_ready or self.bridge_received is None or
                        time.monotonic() - self.bridge_received >= 0.5):
                    self.health_reason = 'motion_sender_disarmed_or_heartbeat_stale'
                    return False
                ready_owners = [(p.node_name, p.node_namespace)
                                for p in self.get_publishers_info_by_topic(BRIDGE_READY)]
                if ready_owners != [('motion_sender', '/lite3_robot_link')]:
                    self.health_reason = 'unexpected_sender_heartbeat_publisher'
                    return False
                peers = self.get_subscriptions_info_by_topic(ROBOT_OUTPUT)
                owners = [(p.node_name, p.node_namespace) for p in peers]
                publishers = self.get_publishers_info_by_topic(ROBOT_OUTPUT)
                inputs = self.get_publishers_info_by_topic(NAV_COMMAND)
                input_owners = [(p.node_name, p.node_namespace) for p in inputs]
                if (owners != [('motion_sender', '/lite3_robot_link')] or
                        input_owners != [('controller_server', '/lite3_autonomy')] or
                        any(p.node_name != self.get_name() or p.node_namespace != '/' for p in publishers)):
                    self.health_reason = 'robot_link_missing_or_unexpected_output_endpoint'
                    return False
            self.health_reason = 'healthy'
            return True

        def command(self, message):
            command = Velocity(message.linear.x, message.linear.y, message.linear.z,
                               message.angular.x, message.angular.y, message.angular.z)
            self.guard.accept(command, self.healthy())
            self.tick()

        def arm(self, request, response):
            healthy = self.healthy()
            was_armed = self.guard.armed
            response.success = self.guard.arm(healthy)
            if response.success and not was_armed:
                self.center = self.pose
                if options.output_mode == 'robot':
                    acknowledgment = String()
                    acknowledgment.data = self.bridge_session
                    self.bridge_ack_pub.publish(acknowledgment)
            response.message = ('Armed; fresh Nav2 commands required' if response.success
                                else 'Not armed: ' + self.health_reason)
            self.tick()
            return response

        def stop(self, request, response):
            self.guard.stop()
            self.tick()
            response.success = True
            response.message = 'Software stop latched; zero requested; no robot mode change or physical acknowledgment'
            return response

        def tick(self):
            command = self.guard.output(self.healthy())
            # Robot mode emits nothing at startup, before its first explicit arm.
            if options.output_mode == 'dry-run' or self.guard.ever_armed:
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = command.linear_x, command.linear_y, command.angular_z
                self.publisher.publish(message)
            enabled = Bool()
            enabled.data = self.guard.armed
            self.enabled_pub.publish(enabled)
            status = String()
            status.data = json.dumps(dict(armed=self.guard.armed, reason=self.guard.reason,
                                          health=self.health_reason, output_mode=options.output_mode))
            self.status_pub.publish(status)

        def shutdown_stop(self):
            from rclpy.duration import Duration
            self.guard.stop('shutdown')
            self.tick()
            self.publisher.wait_for_all_acked(timeout=Duration(seconds=0.2))

    return GuardedAdapter()


def main(args=None):
    options = parse_options(sys.argv[1:] if args is None else args)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    node = executor = None
    try:
        rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
        def interrupt(signum, frame):
            raise KeyboardInterrupt
        signal.signal(signal.SIGTERM, interrupt)
        node = create_node(options, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None and context.ok():
                node.shutdown_stop()
        finally:
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
