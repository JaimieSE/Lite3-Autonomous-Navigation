"""Fixed-topic test node. ROS imports are deferred so policy tests stay offline."""

import argparse
import sys
from typing import Optional, Sequence

from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, Velocity


INPUT_TOPIC = '/lite3_locomotion/test_nav_cmd_vel'
OUTPUT_TOPIC = '/lite3_locomotion/test_cmd_vel'
KILL_SERVICE = '/lite3_locomotion/test_kill'
RESET_KILL_SERVICE = '/lite3_locomotion/test_reset_kill'
PUBLISH_PERIOD = 0.1


def parse_config(args: Optional[Sequence[str]] = None) -> PolicyConfig:
    parser = argparse.ArgumentParser(
        description='Non-actuating fixed-topic velocity test harness; ROS arguments are forbidden.',
        allow_abbrev=False,
    )
    parser.add_argument('--max-linear-x', type=float, default=0.10)
    parser.add_argument('--max-linear-y', type=float, default=0.05)
    parser.add_argument('--max-angular-z', type=float, default=0.20)
    parser.add_argument('--command-timeout', type=float, default=0.50)
    options = parser.parse_args(args)
    try:
        return PolicyConfig(**vars(options))
    except ValueError as error:
        parser.error(str(error))


def create_controller_node(config: PolicyConfig, context):
    """Create endpoints only after checking fixed resolved topic names.

    No caller-supplied topic, namespace, remapping, ROS parameter, or SDK option
    is accepted. Global ROS arguments are ignored even with a supplied context.
    """
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    class LocomotionController(Node):
        def __init__(self):
            super().__init__(
                'lite3_locomotion_controller', namespace='/', context=context,
                use_global_arguments=False, enable_rosout=False,
                start_parameter_services=False,
            )
            if (self.resolve_topic_name(INPUT_TOPIC) != INPUT_TOPIC or
                    self.resolve_topic_name(OUTPUT_TOPIC) != OUTPUT_TOPIC or
                    self.resolve_service_name(KILL_SERVICE) != KILL_SERVICE or
                    self.resolve_service_name(RESET_KILL_SERVICE) != RESET_KILL_SERVICE):
                self.destroy_node()
                raise RuntimeError('Test topic isolation check failed')
            self.policy = CommandPolicy(config)
            self._publisher = self.create_publisher(Twist, OUTPUT_TOPIC, 1)
            self._subscription = self.create_subscription(
                Twist, INPUT_TOPIC, self._on_command, 1,
            )
            self._timer = self.create_timer(PUBLISH_PERIOD, self._publish_output)
            self._kill_service = self.create_service(Trigger, KILL_SERVICE, self._on_kill)
            self._reset_service = self.create_service(Trigger, RESET_KILL_SERVICE, self._on_reset_kill)
            self.get_logger().info(
                f'NON-ACTUATING TEST MODE: {INPUT_TOPIC} -> {OUTPUT_TOPIC}; '
                'no robot bridge or SDK is connected'
            )

        def _on_command(self, message):
            command = Velocity(
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z,
            )
            if not self.policy.accept(command):
                self._publish_output()
                self.get_logger().warning('Command rejected; test output is zero')

        def _publish_output(self):
            command = self.policy.output()
            message = Twist()
            message.linear.x = command.linear_x
            message.linear.y = command.linear_y
            message.angular.z = command.angular_z
            self._publisher.publish(message)

        def stop(self):
            self.policy.set_stop(True)
            self._publish_output()

        def _on_kill(self, request, response):
            self.stop()
            response.success = True
            response.message = 'Test software kill latched; no robot mode change was sent'
            self.get_logger().warning(response.message)
            return response

        def _on_reset_kill(self, request, response):
            self.policy.set_stop(False)
            self._publish_output()
            response.success = True
            response.message = 'Test software kill cleared; a fresh command is required'
            return response

    return LocomotionController()


def main(args: Optional[Sequence[str]] = None) -> None:
    # Reject unknown and ROS arguments before importing or initializing ROS.
    config = parse_config(sys.argv[1:] if args is None else args)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor

    context = Context()
    node = None
    executor = None
    try:
        rclpy.init(args=[], context=context)
        node = create_controller_node(config, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            try:
                if node is not None and context.ok():
                    node.stop()  # Best effort only; shutdown delivery is not guaranteed.
            finally:
                try:
                    if executor is not None:
                        executor.shutdown()
                finally:
                    if node is not None:
                        node.destroy_node()
        finally:
            if context.ok():
                rclpy.shutdown(context=context)
