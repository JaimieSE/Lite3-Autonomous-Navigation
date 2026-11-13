"""Simulated bridge endpoints for isolated dry runs; refuses the live robot domains.

Node name ``motion_sender`` mirrors the vendor subscriber so the same graph
checks run as on the robot. Publishes the bridge telemetry contract at 200 Hz.
"""

import argparse
import math
import os
import time

from .sim_model import FAULTS, SimLite3
from .ros_io import POSTURE_TOGGLE

LIVE_DOMAINS = {8, 64}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fault', choices=FAULTS, default='none')
    parser.add_argument('--start', choices=('lying', 'standing'), default='lying')
    options = parser.parse_args(args)
    domain = int(os.environ.get('ROS_DOMAIN_ID', '0'))
    if domain in LIVE_DOMAINS or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('Simulator refuses live robot domains; run via movement_test dry-run --sim')

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import Bool
    from transfer_interfaces.msg import MotionSimpleCMD

    rclpy.init()
    node = Node('motion_sender')
    model = SimLite3(fault=options.fault, start=options.start)
    clock = time.monotonic
    origin = clock()
    node.create_subscription(Twist, '/cmd_vel', lambda m: model.command_velocity(
        clock() - origin, m.linear.x, m.angular.z), qos_profile_sensor_data)

    def on_simple(msg):
        if msg.cmd_code == POSTURE_TOGGLE and msg.size == 0 and msg.type == 0:
            model.command_toggle(clock() - origin)
    node.create_subscription(MotionSimpleCMD, '/simple_cmd', on_simple, 10)
    joint_pub = node.create_publisher(JointState, '/joint_states', 10)
    odom_pub = node.create_publisher(Odometry, '/leg_odom2', 10)
    imu_pub = node.create_publisher(Imu, '/imu/data', 10)
    handle_pub = node.create_publisher(Twist, '/handle_state', qos_profile_sensor_data)
    alive = {n: node.create_publisher(Bool, f'/sensor_status/{n}_isalive', 10)
             for n in ('imu', 'joint', 'odom2', 'odom', 'ultrasound')}

    def tick():
        model.advance(clock() - origin)
        joints = model.joints()
        if joints:
            msg = JointState()
            msg.name, msg.position = list(joints[0]), list(joints[1])
            joint_pub.publish(msg)
        x, y, z, yaw, vx, vy, wz = model.odom()
        odom = Odometry()
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = x, y, z
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = vx, vy, wz
        odom_pub.publish(odom)
        quat, gyro, acc = model.imu()
        imu = Imu()
        imu.header.frame_id = 'base_link'
        o = imu.orientation
        o.x, o.y, o.z, o.w = quat
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = gyro
        a = imu.linear_acceleration
        a.x, a.y, a.z = acc
        imu_pub.publish(imu)
        handle = Twist()
        handle.linear.x, handle.linear.y, handle.angular.z = model.handle()
        handle_pub.publish(handle)

    def status():
        for pub in alive.values():
            pub.publish(Bool(data=True))

    node.create_timer(0.005, tick)
    node.create_timer(0.5, status)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
