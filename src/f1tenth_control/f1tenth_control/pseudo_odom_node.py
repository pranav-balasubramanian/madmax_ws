#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PseudoOdomNode(Node):
    def __init__(self):
        super().__init__('pseudo_odom_node')

        # Configurable parameters for field calibration
        self.declare_parameter('wheelbase', 0.3)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('drive_topic', '/drive')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.steer = 0.0

        self.last_time = self.get_clock().now()
        self.last_stamp = None  # monotonic TF timestamp guard
        self.br = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        drive_topic = self.get_parameter('drive_topic').value
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            drive_topic,
            self.drive_callback,
            10
        )

        self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def drive_callback(self, msg):
        self.speed = msg.drive.speed
        self.steer = msg.drive.steering_angle

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        if dt <= 0.0 or dt > 1.0:
            if dt > 1.0:
                self.last_time = now
            return

        self.last_time = now

        # Skip if this tick would produce a non-monotonic TF timestamp
        if self.last_stamp is not None and now <= self.last_stamp:
            return
        self.last_stamp = now

        L = self.get_parameter('wheelbase').value
        scale = self.get_parameter('speed_scale').value
        v = self.speed * scale
        delta = self.steer

        # Kinematic bicycle model integration
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += v * math.tan(delta) / L * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        ts = now.to_msg()
        quat = Rotation.from_euler('xyz', [0.0, 0.0, self.theta]).as_quat()

        odom = Odometry()
        odom.header.stamp = ts
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = v * math.cos(self.theta)
        odom.twist.twist.linear.y = v * math.sin(self.theta)
        odom.twist.twist.angular.z = v * math.tan(delta) / L
        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = ts
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = quat[0]
        tf_msg.transform.rotation.y = quat[1]
        tf_msg.transform.rotation.z = quat[2]
        tf_msg.transform.rotation.w = quat[3]
        self.br.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PseudoOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
