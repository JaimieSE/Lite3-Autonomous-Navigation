import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def main():
    rclpy.init()
    node = Node('group8_rgb_preview')
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    output = node.create_publisher(
        CompressedImage, '/group8_perception/rgb/image/compressed', qos)
    next_frame = 0.0

    def forward(frame):
        nonlocal next_frame
        now = time.monotonic()
        if now >= next_frame and output.get_subscription_count():
            output.publish(frame)
            next_frame = now + 0.2

    node.create_subscription(
        CompressedImage, '/camera/color/image_raw/compressed', forward, qos)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
