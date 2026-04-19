#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import Joy, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollower(Node):
    # Forward cone half-angle for emergency stop check (radians)
    ESTOP_HALF_ANGLE = np.radians(15.0)
    ESTOP_DISTANCE = 0.3  # meters

    def __init__(self):
        super().__init__('wall_follower')

        self.declare_parameter('max_speed', 0.75)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('drive_topic', '/drive')

        self.max_speed = self.get_parameter('max_speed').value
        scan_topic = self.get_parameter('scan_topic').value
        drive_topic = self.get_parameter('drive_topic').value

        # Deadman's switch — hold Y button (buttons[3]) to enable
        self.enabled = False

        self.prev_error = 0.0
        self.integral_error = 0.0

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10
        )

        self.get_logger().info(
            f'WallFollower ready. Listening on {scan_topic}, '
            f'publishing to {drive_topic}. Hold Y button to enable.'
        )

    def joy_callback(self, msg):
        # Y button is buttons[3] — hold to enable, release to stop
        if len(msg.buttons) > 3:
            self.enabled = (msg.buttons[3] == 1)
            if not self.enabled:
                self.integral_error = 0.0

    def _emergency_stop(self, rays, min_angle, angle_inc):
        """Return True if any ray within the forward ±15° cone is closer than ESTOP_DISTANCE."""
        num_rays = len(rays)
        center_idx = int((-min_angle) / angle_inc)

        half_span = int(self.ESTOP_HALF_ANGLE / angle_inc) + 1
        lo = max(0, center_idx - half_span)
        hi = min(num_rays, center_idx + half_span + 1)

        cone = rays[lo:hi]
        valid = cone[np.isfinite(cone) & (cone > 0.0)]
        if len(valid) == 0:
            return False
        return bool(np.min(valid) < self.ESTOP_DISTANCE)

    def driving_policy(self, scan):
        rays = scan['rays']
        num_rays = len(rays)
        min_angle = scan['min_angle']
        max_angle = scan['max_angle']
        angle_inc = (max_angle - min_angle) / (num_rays - 1)

        def get_index(target_angle):
            idx = int((target_angle - min_angle) / angle_inc)
            return max(0, min(idx, num_rays - 1))

        # Emergency stop — takes priority over everything
        if self._emergency_stop(rays, min_angle, angle_inc):
            self.integral_error = 0.0
            self.prev_error = 0.0
            return 0.0, 0.0

        # Speed: scale with forward clearance, capped at max_speed
        front_cone = rays[get_index(np.radians(-10)):get_index(np.radians(10)) + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]
        front_dist = float(np.min(valid_front)) if len(valid_front) > 0 else 0.0
        speed = max(0.0, min(front_dist * 2.0, self.max_speed))

        # Steering: compare mean distance to left wall vs right wall at ±60°
        window = 5
        idx_left = get_index(np.radians(60))
        idx_right = get_index(np.radians(-60))

        left_window = rays[max(0, idx_left - window):min(num_rays, idx_left + window + 1)]
        right_window = rays[max(0, idx_right - window):min(num_rays, idx_right + window + 1)]

        valid_left = left_window[np.isfinite(left_window) & (left_window > 0.0)]
        valid_right = right_window[np.isfinite(right_window) & (right_window > 0.0)]

        dist_left = float(np.mean(valid_left)) if len(valid_left) > 0 else 2.0
        dist_right = float(np.mean(valid_right)) if len(valid_right) > 0 else 2.0

        error = dist_left - dist_right

        kp_steer = 0.6
        ki_steer = 0.005
        kd_steer = 0.2

        p_term = kp_steer * error

        self.integral_error += error
        self.integral_error = max(-20.0, min(self.integral_error, 20.0))
        i_term = ki_steer * self.integral_error

        d_term = kd_steer * (error - self.prev_error)
        self.prev_error = error

        steer = p_term + i_term + d_term
        steer = max(-0.4, min(steer, 0.4))

        return float(speed), float(steer)

    def scan_callback(self, msg):
        if not self.enabled:
            return

        scan = {
            'min_angle': float(msg.angle_min),
            'max_angle': float(msg.angle_max),
            'rays': np.array(msg.ranges),
        }

        speed, steer = self.driving_policy(scan)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.drive_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
