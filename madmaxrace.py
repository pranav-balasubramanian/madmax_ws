# Race Code
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped


class MadMaxRace(Node):
    def __init__(self):
        super().__init__("MadMaxRace")
        
        self.enabled = False
        self.start_time = None
        self.startup_duration = 0.33  # seconds
        self.current_speed = 0.0
        self.max_accel = 6.0   # m/s per second — tune this
        self.max_decel = 7.0   # brake faster than you accelerate
        self.prev_time = None


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
                self.start_time = None
                self.prev_time = None
                self.current_speed = 0.0
        
            
    
    def driving_policy(self, scan, dt):
        rays = scan["rays"]
        num_rays = len(rays)
        min_angle = scan["min_angle"]
        max_angle = scan["max_angle"]
        angle_inc = (max_angle - min_angle) / (num_rays - 1)
        
        i = 0
        np_rays = np.array(rays)
        np_rays_ds = np_rays[::3]
        max_variance = 0
        max_window = 0
        while (i < np.size(np_rays_ds)):
            window = np_rays_ds[i : i + 81]
            lower_bound = np.percentile(np_rays_ds, 20)
            upper_bound = np.percentile(np_rays_ds, 80)
            filtered_window = window[(window >= lower_bound) & (window <= upper_bound)]
            window_variance = np.var(filtered_window)
            if (window_variance > max_variance):
                max_variance = window_variance
                max_window = i
            i += 40
        angle_inc_ds = (max_angle - min_angle) / (np.size(np_rays_ds) - 1)
        i = max_window
        #print("Splits at i: ")
        #print(i)
        #print("Splits at degree: ")
        #print(min_angle + i * angle_inc_ds)
        #print("np_rays_ds len:")
        #print(len(np_rays_ds))
        #print("min angle: ")
        #print(min_angle)
        #print("max angle: ")
        #print(max_angle)
        #print("angle_inc_ds: ")
        #print(angle_inc_ds)

        left_wall = np_rays_ds[:i]
        left_degrees = min_angle + np.arange(len(left_wall)) * angle_inc_ds
        right_wall = np_rays_ds[i + 11:]
        right_degrees = min_angle + (i + 11) * angle_inc_ds + np.arange(len(right_wall)) * angle_inc_ds
        left_x = left_wall * np.cos(left_degrees)
        left_y = left_wall * np.sin(left_degrees)
        right_x = right_wall * np.cos(right_degrees)
        right_y = right_wall * np.sin(right_degrees)
        if (np.size(left_wall) > 0 and np.size(right_wall) > 0):
            width = np.min(left_wall) + np.min(right_wall)
        else:
            width = 2.0
        #print("Width: ")
        #print(width)

        def get_index(target_angle):
            idx = int((target_angle - min_angle) / angle_inc)
            return max(0, min(idx, num_rays - 1))
        
        # --- SPEED CONTROL ---
        # Slice the forward cone and filter out invalid/infinite readings
        front_cone = rays[get_index(np.radians(-5)):get_index(np.radians(5)) + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]
        
        front_dist = 0.0
        if len(valid_front) > 0:
            front_dist = np.min(valid_front)
                        
        front_cone_nar = rays[get_index(np.radians(-1)):get_index(np.radians(1)) + 1]
        valid_front_nar = front_cone_nar[np.isfinite(front_cone_nar) & (front_cone_nar > 0.0)]
        front_dist_nar = 0.0
        if len(valid_front_nar) > 0:
            front_dist_nar = np.min(valid_front_nar)
        
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

        #corridor_width = dist_left + dist_right
        #corridor_width = width

        # speed = max(0, min(front_dist * 1.5, 3))
        # speed = 1.5
        if width > 0.2:
            width = 0.075
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.startup_duration:
            target_speed = 5.0
        elif not np.isfinite(front_dist):
            target_speed = 0.8 * width  # fallback when nothing is ahead
        elif front_dist_nar > 13:
	        target_speed = 7.5
        else:
            target_speed = max(1.5, min(front_dist * 1.5, 3.5) * width * 8)
            # #print("Target Speed: ", target_speed)
            # #print("Width: ", width)
            # #print("Front Dist: ", front_dist)
            # #print("Other Speed: ", min(front_dist * 1.5, 3.5) * width)


            # #print(target_speed)
        target_speed = min(3, target_speed)

        if target_speed > self.current_speed:
            self.current_speed += self.max_accel * dt
            self.current_speed = min(self.current_speed, target_speed)
        else:
            self.current_speed -= self.max_decel * dt
            self.current_speed = max(self.current_speed, target_speed)

        speed = self.current_speed
        # --- PID CALCULATIONS ---
        # 1. Proportional term
        kp_steer = 0.4  # Lowered from 0.25 #old 0.6
        p_term = kp_steer * error
        
        # 2. Integral term
        ki_steer = 0.005 # Tune this slowly
        self.integral_error += error
        # Anti-windup: clamp the integral to prevent it from building up too much
        self.integral_error = max(-20.0, min(self.integral_error, 20.0)) 
        i_term = ki_steer * self.integral_error
        
        # 3. Derivative term
        kd_steer = 0.35  # Helps dampen oscillation #0.25 before
        derivative = error - self.prev_error
        d_term = kd_steer * derivative
        
        # Update state for the next loop
        self.prev_error = error
        
        # Combine terms
        steer = p_term + i_term + d_term
        
        # Clamp steering angle to realistic physical limits (e.g., +- 0.4 radians)
        steer = max(-0.4, min(steer, 0.4))
        

        print("Elapsed: ", elapsed)
        print("Speed: ", speed)
        print("Target Speed: ", target_speed)
        print("Steer: ", steer)
        print("Width: ", width)
        print("Front Dist: ", front_dist)
        print("Right/Left Split: ", (min_angle + (i + 5) * angle_inc_ds))

        return float(speed), float(steer)
        


    def scan_callback(self, msg):
        if not self.enabled:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        scan = {
            "min_angle": float(msg.angle_min),
            "max_angle": float(msg.angle_max),
            "rays": np.array(msg.ranges)
        }
        now = self.get_clock().now()

        if self.prev_time is None:
            dt = 0.05  # assume ~20hz on first frame
        else:
            dt = (now - self.prev_time).nanoseconds / 1e9
        self.prev_time = now

        speed, steer = self.driving_policy(scan, dt)
        #print("Speed: ", speed)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = speed
        drive_msg.drive.steering_angle = steer
        self.drive_pub.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MadMaxRace()
    
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
