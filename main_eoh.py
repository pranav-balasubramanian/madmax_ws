import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped


class EOHDemo(Node):
    def __init__(self):
        super().__init__("EOHDemo")
        
        self.enabled = False
        
        # --- PID State Variables ---
        self.prev_error = 0.0
        self.integral_error = 0.0
        
        self.joy_sub = self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )
        
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            10
        )
        
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "/ackermann_cmd",
            10
        )

    def joy_callback(self, msg):
        # enable if "Y" button is pressed
        if len(msg.buttons) > 3:
            self.enabled = (msg.buttons[3] == 1)
            # Reset PID integral when disabled to prevent jerky starts
            if not self.enabled:
                self.integral_error = 0.0
    
    def driving_policy(self, scan):
        rays = scan["rays"]
        num_rays = len(rays)
        min_angle = scan["min_angle"]
        max_angle = scan["max_angle"]
        angle_inc = (max_angle - min_angle) / (num_rays - 1)
        
        def get_index(target_angle):
            idx = int((target_angle - min_angle) / angle_inc)
            return max(0, min(idx, num_rays - 1))
        
        # --- SPEED CONTROL ---
        # Slice the forward cone and filter out invalid/infinite readings
        front_cone = rays[get_index(np.radians(-10)):get_index(np.radians(10)) + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]
        
        front_dist = 0.0
        if len(valid_front) > 0:
            front_dist = np.min(valid_front)
                        
        # speed = max(0, min(front_dist * 1.5, 3))
        speed = 1.5
        
        
        # --- STEERING CONTROL ---
        window = 5
        idx_left = get_index(np.radians(60))
        idx_right = get_index(-np.radians(60))
        
        left_window = rays[max(0, idx_left - window) : min(num_rays, idx_left + window + 1)]
        right_window = rays[max(0, idx_right - window) : min(num_rays, idx_right + window + 1)]
        
        valid_left = left_window[np.isfinite(left_window) & (left_window > 0.0)]
        valid_right = right_window[np.isfinite(right_window) & (right_window > 0.0)]
        
        dist_left = np.mean(valid_left) if len(valid_left) > 0 else 2.0
        dist_right = np.mean(valid_right) if len(valid_right) > 0 else 2.0
        
        error = dist_left - dist_right
        
        # --- PID CALCULATIONS ---
        # 1. Proportional term
        kp_steer = 0.6  # Lowered from 0.25
        p_term = kp_steer * error
        
        # 2. Integral term
        ki_steer = 0.005 # Tune this slowly
        self.integral_error += error
        # Anti-windup: clamp the integral to prevent it from building up too much
        self.integral_error = max(-20.0, min(self.integral_error, 20.0)) 
        i_term = ki_steer * self.integral_error
        
        # 3. Derivative term
        kd_steer = 0.2  # Helps dampen oscillation
        derivative = error - self.prev_error
        d_term = kd_steer * derivative
        
        # Update state for the next loop
        self.prev_error = error
        
        # Combine terms
        steer = p_term + i_term + d_term
        
        # Clamp steering angle to realistic physical limits (e.g., +- 0.4 radians)
        steer = max(-0.4, min(steer, 0.4))
        
        return float(speed), float(steer)

    def scan_callback(self, msg):
        if not self.enabled:
            return

        scan = {
            "min_angle": float(msg.angle_min),
            "max_angle": float(msg.angle_max),
            "rays": np.array(msg.ranges)
        }

        speed, steer = self.driving_policy(scan)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EOHDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = 0.0
        stop_msg.drive.steering_angle = 0.0
        node.drive_pub.publish(stop_msg) # Fixed a minor bug here: changed node.publisher to node.drive_pub
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
