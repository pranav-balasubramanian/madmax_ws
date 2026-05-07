import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped


class MadMaxRace(Node):
    def __init__(self):
        super().__init__("MadMaxRace")
        
        self.enabled = False
        self.tuning_mode = True
        self.print_counter = 0
        # self.start_time = None
        # self.startup_duration = 0.33  # seconds
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
                # self.start_time = None
                self.prev_time = None
                self.current_speed = 0.0
        
            
    
    def driving_policy(self, scan, dt):
        # Input Data
        rays = scan["rays"]
        num_rays = len(rays)
        min_angle = scan["min_angle"]
        max_angle = scan["max_angle"]
        angle_inc = (max_angle - min_angle) / (num_rays - 1)
        
        # Convert angle to index in rays array
        def get_index(target_angle):
          idx = int((target_angle - min_angle) / angle_inc)
          return max(0, min(idx, num_rays - 1))

        # ! Set the fallback to something a bit more reasonable
        def avg_dist_at_angle(angle_deg, window=5, fallback=2.0):
          """
          Average a small LiDAR window around a target angle.
          angle_deg:
              + angle = left side
              - angle = right side
          """
          idx = get_index(np.radians(angle_deg))

          start = max(0, idx - window)
          end = min(num_rays, idx + window + 1)

          ray_window = rays[start:end]
          valid = ray_window[np.isfinite(ray_window) & (ray_window > 0.0)]

          if len(valid) == 0:
              return fallback
          return np.mean(valid)

        def width_at_angle(angle_deg):
          """
          Estimate track width using symmetric left/right LiDAR rays.

          Since the rays are diagonal, multiply by sin(angle)
          to get the sideways width component.

          Used to get width of track for speed control
          """
          left_dist = avg_dist_at_angle(angle_deg)
          right_dist = avg_dist_at_angle(-angle_deg)

          side_scale = np.sin(np.radians(angle_deg))

          return (left_dist + right_dist) * side_scale


        width_near = width_at_angle(60)
        width_mid = width_at_angle(50)
        width_far = width_at_angle(35)

        # Use narrowest width as speed limiter
        lookahead_width = min(width_near, width_mid, width_far)

        if self.tuning_mode:
            self.print_counter += 1
            if self.print_counter % 10 == 0: # used to slow print statements to not flood terminal
                print("\n--- WIDTH TUNING MODE ---")
                print("Width near 60:", width_near)
                print("Width mid 50: ", width_mid)
                print("Width far 35: ", width_far)
                print("Lookahead width:", lookahead_width)
        
        # --- Front Distance ---
        # Slice the forward cone and filter out invalid/infinite readings
        front_cone = rays[get_index(np.radians(-10)):get_index(np.radians(10)) + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]
        
        front_dist = 0.0
        if len(valid_front) > 0:
            front_dist = np.min(valid_front)
                        
        
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
        kd_steer = 0.25  # Helps dampen oscillation
        derivative = error - self.prev_error
        d_term = kd_steer * derivative
        
        # Update state for the next loop
        self.prev_error = error
        
        # Combine terms
        steer = p_term + i_term + d_term
        
        # Clamp steering angle to realistic physical limits (e.g., +- 0.4 radians)
        steer = max(-0.4, min(steer, 0.4))



        # --- Speed Control --- 
        min_speed = 1.3
        max_speed = 3.0

        # ! Tune these based on real measurements from your track
        # At min_width, car drives min_speed
        # At max_width, car drives max_speed
        min_width = 0.8
        max_width = 1.6

        # Convert lookahead_width into a 0.0 to 1.0 ratio
        # 0.0 means narrowest expected track
        # 1.0 means widest expected track
        width_ratio = (lookahead_width - min_width) / (max_width - min_width)

        # Clip ratio so bad LiDAR readings don't cause insane speeds
        width_ratio = max(0.0, min(width_ratio, 1.0))

        # Linearly map width to speed
        width_based_speed = min_speed + width_ratio * (max_speed - min_speed)

        # --- Turn Slowdown ---
        # Larger steering angle -> lower speed.
        # Tune turn_slowdown_gain:
        #   0.0 = no slowdown in turns
        #   0.2 = mild slowdown
        #   0.4 = stronger slowdown
        #   0.6 = aggressive slowdown
        turn_slowdown_gain = 0.01 # ! tune this. Increase if car going too fast in turns
        # min_turn_scale = 0.65
        max_steer = 0.4

        steer_ratio = abs(steer) / max_steer
        steer_ratio = max(0.0, min(steer_ratio, 1.0))

        turn_scale = 1.0 - turn_slowdown_gain * steer_ratio
        # turn_scale = max(min_turn_scale, turn_scale)

        turn_based_speed = width_based_speed * turn_scale

        # Front-distance protection
        # This keeps the car from going too fast if something is close ahead.
        if np.isfinite(front_dist) and front_dist > 0.0:
            front_based_speed = min(front_dist * 1.3, max_speed)
            target_speed = min(turn_based_speed, front_based_speed)
        else:
            target_speed = turn_based_speed

        # Final safety clamp
        target_speed = max(min_speed, min(target_speed, max_speed))


        # Smooth speed changes (increment only by a certain amount)
        if target_speed > self.current_speed:
            self.current_speed += self.max_accel * dt
            self.current_speed = min(self.current_speed, target_speed)
        else:
            self.current_speed -= self.max_decel * dt
            self.current_speed = max(self.current_speed, target_speed)

        speed = self.current_speed
        
        return float(speed), float(steer)

    def scan_callback(self, msg):
        if not self.enabled:
            return

        # if self.start_time is None:
        #     self.start_time = self.get_clock().now()

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
        
        if self.tuning_mode:
            speed = 0.0
            steer = 0.0
            self.current_speed = 0.0
            self.prev_time = None
        
        print("Speed: ", speed)

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
