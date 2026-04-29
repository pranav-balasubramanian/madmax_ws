#!/usr/bin/env python3
"""Publish one /initialpose message to seed AMCL.

Usage:
  ros2 run f1tenth_control set_pose <x> <y> <yaw_radians>

Example:
  ros2 run f1tenth_control set_pose 1.20 -0.45 1.5708
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main(args=None):
    if len(sys.argv) < 4:
        print('Usage: set_pose <x> <y> <yaw_radians>', file=sys.stderr)
        sys.exit(1)
    x, y, yaw = (float(s) for s in sys.argv[1:4])

    rclpy.init(args=args)
    node = Node('set_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    qx, qy, qz, qw = yaw_to_quat(yaw)
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw
    cov = [0.0] * 36
    cov[0] = 0.25    # x  (~0.5 m std)
    cov[7] = 0.25    # y
    cov[35] = 0.07   # yaw (~15° std)
    msg.pose.covariance = cov

    # Wait briefly for AMCL to subscribe; otherwise the message is dropped.
    deadline = time.time() + 2.0
    while time.time() < deadline and pub.get_subscription_count() == 0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().warn('No subscribers on /initialpose; publishing anyway.')

    pub.publish(msg)
    node.get_logger().info(
        f'Published /initialpose: x={x:+.2f}, y={y:+.2f}, yaw={math.degrees(yaw):+.1f}°'
    )
    rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
