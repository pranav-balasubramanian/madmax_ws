# F1Tenth SLAM Mapping — Technical Handoff

---

## 1. Project Overview

### What It Does

This project runs a wall-following controller on an F1Tenth autonomous racecar inside the `f1tenth_gym_ros` ROS2 simulator, while simultaneously building a 2D occupancy grid map of the track using `slam_toolbox`. The result is a saved `.pgm` image and a reloadable pose-graph session that can be used as the basis for localization-only (AMCL/EKF) or racing-line planning in later development phases.

### The Full Pipeline

```
gym_bridge (simulator)
  │
  ├─► /ego_racecar/scan  (LaserScan, 250 Hz)  ──────────────► slam_toolbox
  │                                                               │
  └─► /ego_racecar/drive (AckermannDriveStamped) ◄──────────  wall-follower (sim_main.py)
              │
              └─► pseudo_odom_node
                      │
                      ├─► /odom  (nav_msgs/Odometry, 50 Hz)
                      └─► TF: odom → ego_racecar/base_link (50 Hz)

gym_bridge also publishes:
  TF: ego_racecar/base_link → ego_racecar/laser  (250 Hz, always)

slam_toolbox publishes:
  /map  (nav_msgs/OccupancyGrid, ~0.5 Hz while mapping)
  TF: map → odom  (50 Hz once scans are processing)
```

The robot has **no wheel encoders and no IMU**. The only motion signal available is the commanded speed and steering angle on `/ego_racecar/drive`. `pseudo_odom_node` integrates those commands through a kinematic bicycle model to produce a dead-reckoning odometry estimate. This is intentionally imperfect — slam_toolbox's scan-matching corrects for drift.

### What Working Looks Like

- In RViz2, a top-down grey occupancy grid grows as the car drives. Black pixels are walls, white pixels are free space, grey is unmapped.
- The laser scan (red dots) tracks the car's current position inside the growing map.
- `ros2 topic hz /map` prints approximately 0.5 Hz.
- No `LaserRangeScan contains 1080 range readings, expected 1081` errors in the console.
- Running `save_map.sh` produces four files: `.data`, `.posegraph`, `.pgm`, `.yaml`.

---

## 2. Full Environment Specification

### OS and ROS2

| Item | Version |
|---|---|
| OS | Ubuntu 22.04 (also tested under WSL2 on Windows 11) |
| ROS2 | Humble Hawksbill |
| Python | 3.10 (ships with Ubuntu 22.04) |
| Simulator | f1tenth_gym_ros (MIT, Hongrui Zheng) |

### ROS2 Package Dependencies

Install all of these before building:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-ackermann-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard
```

### Python Dependencies

```bash
pip3 install \
  numpy \
  scipy \
  Pillow \
  transforms3d \
  pyyaml \
  gym==0.19.0 \
  numba \
  pyglet==1.5.27 \
  pyopengl
```

> **Note on `gym` version:** f1tenth_gym requires `gym==0.19.0` exactly. Newer versions of OpenAI Gym have breaking API changes. If you have a newer version installed, `pip3 install gym==0.19.0` will downgrade it.

> **Note on `pyglet` version:** `pyglet<1.5` is required by f110_gym. `pyglet==1.5.27` is the last 1.x release.

### Simulator Source

The simulator is `f1tenth_gym_ros`, cloned into the workspace:

```bash
mkdir -p ~/sim/src/f1tenth_simulator
cd ~/sim/src/f1tenth_simulator
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
# The gym Python backend is a submodule or separate pip install:
git clone https://github.com/f1tenth/f1tenth_gym.git
cd f1tenth_gym && pip3 install -e .
```

### Directory Tree

```
~/sim/
├── HANDOFF.md                        ← this file
├── main.py                           ← wall-follower for the REAL vehicle (not sim)
├── sim_main.py                       ← wall-follower adapted for the simulator
├── save_map.sh                       ← saves map session + viewable PGM
├── save_pgm.py                       ← Python helper: subscribes /map → writes .pgm/.yaml
│
└── src/f1tenth_simulator/
    └── f1tenth_gym_ros/              ← ROS2 package (ament_python)
        ├── package.xml
        ├── setup.py                  ← MODIFIED: adds pseudo_odom_node entry point
        ├── resource/f1tenth_gym_ros
        │
        ├── config/
        │   ├── sim.yaml              ← simulator parameters (map, scan FOV, etc.)
        │   └── mapper_params.yaml    ← slam_toolbox tuning (NEW)
        │
        ├── launch/
        │   ├── gym_bridge_launch.py  ← original simulator-only launch
        │   ├── slam_mapping.launch.py← full SLAM stack launch (NEW)
        │   ├── slam_rviz.rviz        ← RViz2 config for SLAM view (NEW)
        │   ├── ego_racecar.xacro
        │   └── opp_racecar.xacro
        │
        ├── maps/
        │   ├── Spielberg_map.png
        │   ├── Spielberg_map.yaml
        │   └── ...
        │
        └── f1tenth_gym_ros/
            ├── __init__.py
            ├── gym_bridge.py         ← MODIFIED: publish_tf param + angle_inc fix
            └── pseudo_odom_node.py   ← kinematic bicycle model odometry (NEW)
```

---

## 3. Every File — Full Contents and Explanation

---

### `~/sim/sim_main.py` — Wall-Follower Controller (Simulator)

```python
import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class SimEOHDemo(Node):
    def __init__(self):
        super().__init__("SimEOHDemo")

        self.enabled = True

        self.declare_parameter('max_speed', 1.5)
        self.max_speed = self.get_parameter('max_speed').value

        # --- PID State Variables ---
        self.prev_error = 0.0
        self.integral_error = 0.0

        self.lidar_sub = self.create_subscription(
            LaserScan,
            "ego_racecar/scan",
            self.scan_callback,
            10
        )

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            "ego_racecar/drive",
            10
        )

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
        front_cone = rays[get_index(np.radians(-10)):get_index(np.radians(10)) + 1]
        valid_front = front_cone[np.isfinite(front_cone) & (front_cone > 0.0)]

        front_dist = 0.0
        if len(valid_front) > 0:
            front_dist = np.min(valid_front)

        speed = max(0.0, min(front_dist * 2, self.max_speed))

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
        kp_steer = 0.6
        p_term = kp_steer * error

        ki_steer = 0.005
        self.integral_error += error
        self.integral_error = max(-20.0, min(self.integral_error, 20.0))
        i_term = ki_steer * self.integral_error

        kd_steer = 0.2
        derivative = error - self.prev_error
        d_term = kd_steer * derivative

        self.prev_error = error

        steer = p_term + i_term + d_term
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
    node = SimEOHDemo()

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
```

**What it does:** A PID wall-follower that keeps equal distance to the left and right walls by comparing LiDAR range readings at ±60°. Speed scales linearly with forward clearance.

**Key design decisions:**

- **Topic names:** Subscribes to `ego_racecar/scan` and publishes to `ego_racecar/drive`. In the simulator, gym_bridge publishes the LiDAR under the ego namespace. The drive topic name `ego_racecar/drive` matches the remapping in `gym_bridge_launch.py` (`/drive` → `ego_racecar/drive`), which means gym_bridge's internal subscription is reached at the fully-resolved topic name.
- **`max_speed` parameter:** Added so the SLAM launch can cap speed at 0.5 m/s without modifying this file. The real-vehicle version (`main.py`) hardcodes 1.5 m/s. In the SLAM launch this is overridden via `--ros-args -p max_speed:=0.5`. Lower speed gives slam_toolbox more time to match scans and reduces map smearing.
- **`angle_inc` computation:** Uses `(max_angle - min_angle) / (num_rays - 1)` to compute angles from scan header values, which is the correct formula for N points inclusive on both endpoints. This makes the wall-follower tolerant to any scan configuration.
- **PID tuning:** `kp=0.6`, `ki=0.005`, `kd=0.2`. The integral anti-windup clamp (±20) prevents the car from overcorrecting after being stuck near a wall. These values were tuned for the Spielberg map at 1.5 m/s; at 0.5 m/s for SLAM they are intentionally over-damped.

---

### `f1tenth_gym_ros/pseudo_odom_node.py` — Kinematic Bicycle Model Odometry

```python
import math

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class PseudoOdomNode(Node):
    WHEELBASE = 0.3  # meters

    def __init__(self):
        super().__init__('pseudo_odom_node')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.speed = 0.0
        self.steer = 0.0

        self.last_time = self.get_clock().now()
        self.last_stamp = None   # enforces strictly-monotonic TF timestamps
        self.br = TransformBroadcaster(self)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.drive_sub = self.create_subscription(
            AckermannDriveStamped,
            'ego_racecar/drive',
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
            # On a large gap (startup) advance the reference; on backwards jitter keep it
            if dt > 1.0:
                self.last_time = now
            return

        self.last_time = now

        # WSL2 clock can jitter: skip if this tick would produce a non-monotonic TF
        if self.last_stamp is not None and now <= self.last_stamp:
            return
        self.last_stamp = now

        v = self.speed
        delta = self.steer
        L = self.WHEELBASE

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += v * math.tan(delta) / L * dt
        # normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        ts = now.to_msg()
        quat = Rotation.from_euler('xyz', [0.0, 0.0, self.theta]).as_quat()

        odom = Odometry()
        odom.header.stamp = ts
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'ego_racecar/base_link'
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
        tf_msg.child_frame_id = 'ego_racecar/base_link'
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
```

**What it does:** Subscribes to drive commands, integrates them through a kinematic bicycle model, and publishes both a `nav_msgs/Odometry` message on `/odom` and a TF transform `odom → ego_racecar/base_link`. This is the motion estimate that slam_toolbox uses to predict where the robot moved between scans.

**Why this exists:** The F1Tenth hardware has no wheel encoders and no IMU. The only motion information available from the simulator is the commanded speed and steering angle. Real odometry would come from wheel tick counts; we substitute commanded velocity as a proxy. Because the simulator's physics engine does execute these commands faithfully (no wheel slip in the default gym), the pseudo-odometry is actually quite accurate inside the simulator — more accurate than it would be on real hardware.

**Kinematic bicycle model:**

```
ẋ     = v · cos(θ)
ẏ     = v · sin(θ)
θ̇     = v · tan(δ) / L
```

Where `v` = commanded speed (m/s), `δ` = commanded steering angle (rad), `L` = wheelbase = 0.3 m (the F1Tenth 1:10 scale car).

**Why 50 Hz timer:** slam_toolbox's scan message filter needs a TF transform at the scan timestamp (or within `transform_timeout` of it). The simulator publishes scans at 250 Hz. Publishing TF at 50 Hz means the maximum gap between a scan timestamp and the nearest TF is 20 ms, well within the 500 ms `transform_timeout`. Publishing faster (e.g. 250 Hz) wastes CPU; publishing slower increases the chance of TF lookup misses.

**Why the monotonic stamp guard (`last_stamp`):** WSL2's system clock is notoriously unreliable. Windows' time synchronization with the WSL2 VM can cause the system clock to jump backwards by a small amount. This produces a `dt < 0` on the next timer tick. Without the guard, two consecutive TF messages could have the same or a decreasing timestamp, causing TF2 to log `TF_OLD_DATA` warnings and discard the second message. The guard simply skips any tick where `now <= last_stamp`.

**Why `dt > 1.0` resets but `dt < 0` does not:** A large positive `dt` means the timer fired late (normal on a loaded system) or this is the first tick after startup. Advancing `last_time` to `now` is safe — we just skip integrating over the gap. A negative `dt` means the clock went backwards; advancing `last_time` would cause the *next* `dt` to be even more negative. So we leave `last_time` alone and retry on the next tick.

**Angle normalization:** `theta = atan2(sin(theta), cos(theta))` wraps the heading to `(-π, π]` after each integration step, preventing unbounded accumulation.

---

### `f1tenth_gym_ros/gym_bridge.py` — Simulator Bridge (Modified)

The full original file is not reproduced here because it is 424 lines of upstream code. Only the two modifications made to the original are documented.

**Modification 1 — `publish_tf` parameter (lines ~74 and ~271)**

In `__init__`, after `self.declare_parameter('kb_teleop', True)`:

```python
self.declare_parameter('publish_tf', True)
```

In `timer_callback`, the call to `_publish_transforms` is gated:

```python
# Before (original):
self._publish_transforms(ts)

# After (modified):
if self.get_parameter('publish_tf').value:
    self._publish_transforms(ts)
```

**Why:** `_publish_transforms` broadcasts the TF `map → ego_racecar/base_link` using ground-truth pose from the simulator physics engine. In the SLAM launch we set `publish_tf: False` to disable this, because slam_toolbox must own the `map → odom` transform and pseudo_odom must own `odom → base_link`. If gym_bridge also broadcasts `map → base_link` directly, TF2 sees `base_link` with two parents (`map` via gym_bridge, and `odom` via pseudo_odom), which is an illegal cycle that breaks all TF lookups.

The other two TF methods — `_publish_laser_transforms` (which broadcasts `base_link → laser`) and `_publish_wheel_transforms` — are **not** gated, because slam_toolbox needs the `base_link → laser` transform to project scan points into the base frame.

**Modification 2 — `angle_inc` formula (line ~110)**

```python
# Before (original):
self.angle_inc = scan_fov / scan_beams

# After (modified):
self.angle_inc = scan_fov / (scan_beams - 1)
```

**Why:** This was the root cause of slam_toolbox never building a map. The original formula treats `scan_beams` as the number of intervals, giving `scan_beams` samples covering the full FOV. But the correct formula for `N` evenly-spaced samples spanning `[angle_min, angle_max]` inclusive is `angle_inc = FOV / (N - 1)` (N points, N-1 gaps).

slam_toolbox internally computes the expected number of ranges as:

```
expected = round((angle_max - angle_min) / angle_increment) + 1
```

With the original formula: `expected = round(4.7 / (4.7/1080)) + 1 = 1080 + 1 = 1081`, but gym_bridge sends 1080. slam_toolbox logs `LaserRangeScan contains 1080 range readings, expected 1081` and **silently returns null**, discarding the scan. Every scan is discarded, so `/map` is never published.

With the fix: `angle_inc = 4.7/1079`, so `expected = round(4.7 / (4.7/1079)) + 1 = 1079 + 1 = 1080`. This matches the actual scan length.

---

### `config/sim.yaml` — Simulator Parameters

```yaml
bridge:
  ros__parameters:
    ego_namespace: 'ego_racecar'
    ego_scan_topic: 'scan'
    ego_odom_topic: 'odom'
    ego_opp_odom_topic: 'opp_odom'
    ego_drive_topic: 'drive'
    opp_namespace: 'opp_racecar'
    opp_scan_topic: 'opp_scan'
    opp_odom_topic: 'odom'
    opp_ego_odom_topic: 'opp_odom'
    opp_drive_topic: 'opp_drive'

    scan_distance_to_base_link: 0.0

    scan_fov: 4.7
    scan_beams: 1080

    map_path: 'Spielberg_map'
    map_img_ext: '.png'

    num_agent: 1

    sx: 0.0
    sy: 0.0
    stheta: 0.0

    sx1: 2.0
    sy1: 0.5
    stheta1: 0.0

    kb_teleop: True
```

**Key parameters:**

- `map_path: 'Spielberg_map'` — the track used for all SLAM runs. This is resolved relative to the package share directory at `maps/Spielberg_map.png`. Change this to use a different track (e.g. `levine`, `square`).
- `scan_fov: 4.7` — the LiDAR field of view in radians (~269°), symmetric around forward.
- `scan_beams: 1080` — the number of range readings per scan.
- `scan_distance_to_base_link: 0.0` — offset of the laser from base_link along the x-axis in meters. The F1Tenth car has the LiDAR at the front; set this to the physical offset if running on real hardware.
- `num_agent: 1` — single car mode. Setting to 2 adds an opponent.

---

### `config/mapper_params.yaml` — slam_toolbox Configuration

```yaml
slam_toolbox:
  ros__parameters:

    scan_topic: /ego_racecar/scan
    base_frame: ego_racecar/base_link
    odom_frame: odom
    map_frame: map

    mode: mapping

    solver_plugin: solver_plugins::CeresSolver
    ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
    ceres_preconditioner: SCHUR_JACOBI
    ceres_trust_strategy: LEVENBERG_MARQUARDT
    ceres_dogleg_type: TRADITIONAL_DOGLEG
    ceres_loss_function: None

    resolution: 0.05
    max_laser_range: 15.0

    minimum_time_interval: 0.5
    minimum_travel_distance: 0.05
    minimum_travel_heading: 0.1

    transform_publish_period: 0.02
    map_update_interval: 2.0

    transform_timeout: 0.5
    tf_buffer_duration: 30.0

    throttle_scans: 5

    use_scan_matching: true
    use_scan_barycenter: true
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5

    do_loop_closing: true
    loop_search_maximum_distance: 4.0
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45

    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1

    loop_search_space_dimension: 8.0
    loop_search_space_resolution: 0.05
    loop_search_space_smear_deviation: 0.03

    distance_variance_penalty: 0.5
    angle_variance_penalty: 1.0
    fine_search_angle_offset: 0.00349
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true

    stack_size_to_use: 40000000
    enable_interactive_mode: true
    debug_logging: false
```

**Key tuning decisions:**

- `resolution: 0.05` — 5 cm grid cells. Fine enough to see track boundaries clearly; coarser values (0.1) produce blurry maps.
- `max_laser_range: 15.0` — Spielberg map is roughly 20 m across; 15 m captures most walls without processing returns from outside the track bounds.
- `minimum_time_interval: 0.5` — Accept a new keyframe at most every 500 ms. At 0.5 m/s the car moves 0.25 m between keyframes, which is sufficient scan overlap.
- `throttle_scans: 5` — Process every 5th scan (gym_bridge publishes at 250 Hz → effective 50 Hz). Prevents slam_toolbox from being overwhelmed and avoids the message filter queue filling up.
- `transform_timeout: 0.5` — slam_toolbox waits up to 500 ms for a TF at the scan timestamp. This large value is intentional for WSL2, where clock jitter can delay TF availability.
- `tf_buffer_duration: 30.0` — Keep 30 seconds of TF history. Large value ensures slam_toolbox can still look up old transforms when processing queued scans.
- `do_loop_closing: true` — Enabled so that when the car completes a lap, slam_toolbox detects the revisited location and corrects accumulated drift.

---

### `launch/slam_mapping.launch.py` — Full SLAM Stack Launch

```python
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pkg = get_package_share_directory('f1tenth_gym_ros')

    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to suppress RViz2'
    ))

    sim_config = os.path.join(pkg, 'config', 'sim.yaml')
    mapper_params = os.path.join(pkg, 'config', 'mapper_params.yaml')
    rviz_config = os.path.join(pkg, 'launch', 'slam_rviz.rviz')

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[sim_config, {'publish_tf': False}],
        remappings=[('/drive', 'ego_racecar/drive'),
                    ('/odom', 'ego_racecar/odom')],
        output='screen'
    )

    wall_follower = ExecuteProcess(
        cmd=[
            'python3', '/home/pranav/sim/sim_main.py',
            '--ros-args', '-p', 'max_speed:=0.5'
        ],
        output='screen'
    )

    pseudo_odom = Node(
        package='f1tenth_gym_ros',
        executable='pseudo_odom_node',
        name='pseudo_odom_node',
        output='screen'
    )

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_params],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    ld.add_action(bridge_node)
    ld.add_action(wall_follower)
    ld.add_action(pseudo_odom)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    return ld
```

**Design decisions:**

- `publish_tf: False` on the bridge node — see gym_bridge.py explanation above.
- `wall_follower` uses `ExecuteProcess` (not `Node`) because `sim_main.py` lives outside the ROS2 package directory and cannot be registered as an entry point without moving it. `ExecuteProcess` runs it as a plain Python process. The `--ros-args -p max_speed:=0.5` passes a ROS2 parameter via `sys.argv`, which `rclpy.init()` picks up automatically.
- The `nav2_map_server` that appears in `gym_bridge_launch.py` is **intentionally omitted** here. That server publishes the pre-built simulator map on `/map`. If it were included, there would be two publishers on `/map` (nav2_map_server and slam_toolbox), which would cause rviz to show whichever message arrived last — potentially the static simulator map overwriting the live SLAM map.
- `async_slam_toolbox_node` is used instead of `sync_slam_toolbox_node`. The async variant processes scans in a background thread and does not block the ROS2 executor, which means drive commands and TF publishing continue at full rate even when slam_toolbox is doing expensive loop closure.

---

### `launch/slam_rviz.rviz` — RViz2 Configuration

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map (SLAM)
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Value: true
    - Class: rviz_default_plugins/LaserScan
      Color: 213; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Name: LaserScan
      Position Transformer: XYZ
      Size (m): 0.02
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /ego_racecar/scan
      Value: true
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Name: TF
      Show Arrows: false
      Show Axes: true
      Show Names: true
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/TopDownOrtho
      Name: Current View
      Scale: 50
      Value: true
    Saved: ~
```

**Key settings:**

- `Fixed Frame: map` — all displays are shown in the slam_toolbox map coordinate frame.
- `Durability Policy: Transient Local` on the `/map` display — slam_toolbox publishes the occupancy grid with transient-local (latched) QoS. RViz2 must match this policy or it will never receive the map on startup.
- `Durability Policy: Volatile` on `/ego_racecar/scan` — scans are published volatile (no latching). Using Transient Local here would cause RViz2 to try to receive old scans and fail.
- `TopDownOrtho` view at `Scale: 50` — a top-down 2D orthographic view appropriate for a floor map. Scale 50 means 50 pixels per meter.

---

### `launch/slam_mapping.launch.py` — (documented above)

---

### `setup.py` — Package Entry Points (Modified)

```python
from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_gym_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.xacro')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Billy Zheng',
    maintainer_email='billyzheng.bz@gmail.com',
    description='Bridge for using f1tenth_gym in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gym_bridge = f1tenth_gym_ros.gym_bridge:main',
            'pseudo_odom_node = f1tenth_gym_ros.pseudo_odom_node:main',
        ],
    },
)
```

**Modification:** Added `'pseudo_odom_node = f1tenth_gym_ros.pseudo_odom_node:main'` to `console_scripts`. This registers `pseudo_odom_node` as a system command in the ROS2 install prefix, allowing the launch file to reference it via `executable='pseudo_odom_node'` with `package='f1tenth_gym_ros'`. Without this entry, the launch file cannot find the node and it silently fails to start.

**Important:** Unlike Python source files, entry point changes require a full `colcon build` (not just sourcing). `--symlink-install` does not update entry points on its own.

---

### `~/sim/save_map.sh` — Map Saving Script

```bash
#!/bin/bash
set -e

MAP_DIR="$HOME/f1tenth_maps"
MAP_NAME="track_map"
FULL_PATH="$MAP_DIR/$MAP_NAME"

mkdir -p "$MAP_DIR"

echo "==> Serializing slam_toolbox pose graph (reloadable session)..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$FULL_PATH'}"

echo ""
echo "==> Saving viewable occupancy grid (subscribes /map with Transient Local QoS)..."
python3 "$(dirname "$0")/save_pgm.py" "$FULL_PATH"

echo ""
echo "Files saved to $MAP_DIR:"
echo "  Reloadable session : ${FULL_PATH}.data"
echo "                       ${FULL_PATH}.posegraph"
echo "  Viewable map       : ${FULL_PATH}.pgm"
echo "                       ${FULL_PATH}.yaml"
```

**What it produces:**

| File | Purpose |
|---|---|
| `track_map.data` | slam_toolbox serialized scan data |
| `track_map.posegraph` | slam_toolbox pose graph with loop closure edges |
| `track_map.pgm` | Viewable grayscale image (black=wall, white=free, grey=unknown) |
| `track_map.yaml` | Map metadata for nav2 (resolution, origin, thresholds) |

The `.data` + `.posegraph` pair can be reloaded into slam_toolbox for continued mapping or for localization-only mode. The `.pgm` + `.yaml` pair is what nav2 stack consumers (AMCL, navfn, etc.) read.

---

### `~/sim/save_pgm.py` — Map Topic Subscriber / PGM Writer

```python
#!/usr/bin/env python3
"""Subscribe to /map with Transient Local QoS and write .pgm + .yaml."""
import struct
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapSaver(Node):
    def __init__(self, output_path):
        super().__init__('map_pgm_saver')
        self.output_path = output_path
        self.done = False

        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb, qos)
        self.get_logger().info('Waiting for /map ...')

    def _cb(self, msg):
        if self.done:
            return
        self.done = True

        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        pgm_path = self.output_path + '.pgm'
        yaml_path = self.output_path + '.yaml'

        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{w} {h}\n255\n'.encode())
            for row in range(h - 1, -1, -1):
                for col in range(w):
                    val = msg.data[row * w + col]
                    if val == -1:
                        pixel = 205
                    elif val == 0:
                        pixel = 254
                    else:
                        pixel = 0
                    f.write(struct.pack('B', pixel))

        with open(yaml_path, 'w') as f:
            f.write(f'image: {pgm_path}\n')
            f.write(f'resolution: {res}\n')
            f.write(f'origin: [{ox}, {oy}, 0.0]\n')
            f.write('negate: 0\n')
            f.write('occupied_thresh: 0.65\n')
            f.write('free_thresh: 0.25\n')

        self.get_logger().info(f'Saved {pgm_path}')
        self.get_logger().info(f'Saved {yaml_path}')
        rclpy.shutdown()


def main():
    output = sys.argv[1] if len(sys.argv) > 1 else '/tmp/map'
    rclpy.init()
    rclpy.spin(MapSaver(output))


if __name__ == '__main__':
    main()
```

**Why this exists instead of `nav2_map_server map_saver_cli`:** slam_toolbox publishes `/map` with `Transient Local` (latched) QoS. `map_saver_cli` subscribes with `Volatile` QoS by default. In DDS, a Transient Local publisher and a Volatile subscriber are incompatible — the subscriber never receives the latched message and times out after 5 seconds with "Failed to spin map subscription". The `map_subscribe_transient_local` parameter exists in nav2's `map_saver` node but is not reliably picked up by the `map_saver_cli` wrapper in all Humble builds.

This Python script explicitly constructs a `QoSProfile` with `TRANSIENT_LOCAL` durability, guaranteeing it receives the latched map message immediately upon subscription.

**Row flipping:** The PGM format has the origin at the top-left corner. ROS2 `OccupancyGrid` has the origin at the bottom-left. The `range(h - 1, -1, -1)` loop reverses row order when writing so the saved image displays correctly (north up, not north down).

**Pixel encoding:** `val == -1` (unknown) → 205 (grey). `val == 0` (free) → 254 (near-white). `val > 0` (occupied) → 0 (black). This matches the nav2 standard for `.pgm` map files.

---

## 4. Known Issues and Fixes Applied

### Issue 1 — Wrong Topic Names (Simulator vs. Real Vehicle)

**Symptom:** Wall-follower runs on real vehicle fine, does nothing in simulator.

**Cause:** The real vehicle uses `/scan` (LaserScan) and `/ackermann_cmd` (drive commands). The simulator uses `ego_racecar/scan` and `ego_racecar/drive`.

**Fix:** `sim_main.py` was derived from `main.py` with the following changes:
- Subscribe `ego_racecar/scan` (not `/scan`)
- Publish `ego_racecar/drive` (not `/ackermann_cmd`)
- Remove the joystick enable/disable (simulator is always enabled)
- Add `max_speed` parameter

---

### Issue 2 — TF Tree Conflict: gym_bridge vs. slam_toolbox Both Own `map → base_link`

**Symptom:** When both `gym_bridge` and slam_toolbox are running, TF2 logs errors about cycles or dendritic trees, and slam_toolbox cannot find valid TF lookups.

**Cause:** gym_bridge's `_publish_transforms()` broadcasts `map → ego_racecar/base_link` at 250 Hz as ground-truth pose. slam_toolbox needs to own the chain `map → odom → ego_racecar/base_link`. In TF2, a frame (`ego_racecar/base_link`) can only have one parent. Having two separate paths (`map → base_link` directly, and `map → odom → base_link`) creates an illegal tree.

**Fix:** Added `publish_tf` parameter to gym_bridge. The SLAM launch passes `publish_tf: False`, disabling `_publish_transforms` while leaving `_publish_laser_transforms` (which gym_bridge must always publish for scan matching to work).

---

### Issue 3 — slam_toolbox Silently Rejects Every Scan (Off-By-One Beam Count)

**Symptom:** `/map` never publishes. Console logs: `LaserRangeScan contains 1080 range readings, expected 1081`. `ros2 topic hz /map` shows 0 Hz.

**Cause:** gym_bridge computed `angle_inc = scan_fov / scan_beams = 4.7 / 1080`. slam_toolbox computes its expected beam count as `round((angle_max - angle_min) / angle_inc) + 1 = 1081`. The mismatch causes slam_toolbox to return `null` for every scan and skip it entirely.

**Fix:** Changed gym_bridge to `angle_inc = scan_fov / (scan_beams - 1)`. With N points spanning a range inclusively, there are N−1 gaps, so the increment is `FOV / (N−1)`. Now `expected = round(FOV / (FOV/(N-1))) + 1 = (N-1) + 1 = N`. Matches.

---

### Issue 4 — slam_toolbox `save_map` Service Returns `result=1`, No `.pgm` Created

**Symptom:** `ros2 service call /slam_toolbox/save_map` responds with `result=1`. No `.pgm` or `.yaml` files appear.

**Cause:** slam_toolbox's internal `save_map` service returns 1 (failure) when the `OccupancyGrid` map object has not yet been computed. This happens when the car hasn't driven enough for slam_toolbox to publish its first `/map` message (map is built on an interval, not every scan).

**Fix:** Replaced the `save_map` service call with `save_pgm.py`, which subscribes directly to the `/map` topic that slam_toolbox publishes on its update interval. Once the car has driven enough and slam_toolbox has published at least one map, `save_pgm.py` receives it immediately via Transient Local QoS and writes the files.

---

### Issue 5 — `map_saver_cli` Fails with "Failed to spin map subscription"

**Symptom:** `ros2 run nav2_map_server map_saver_cli -f ...` exits with error after a 5-second timeout.

**Cause:** slam_toolbox publishes `/map` with `Transient Local` (latched) durability. `map_saver_cli` in ROS2 Humble subscribes with `Volatile` durability by default. A `Volatile` subscriber cannot receive messages from a `Transient Local` publisher — DDS considers them incompatible. The `map_subscribe_transient_local` parameter exists but is not reliably picked up by the CLI wrapper.

**Fix:** `save_pgm.py` (see above) constructs the subscription with `TRANSIENT_LOCAL` durability explicitly in Python code.

---

### Issue 6 — `TF_OLD_DATA` Warning Spam at Startup

**Symptom:** Console floods with `TF_OLD_DATA ignoring data from the past for frame odom at time <fixed_timestamp>`. The same fixed timestamp repeats indefinitely.

**Cause:** slam_toolbox publishes an initial identity `map → odom` transform at node startup time. This initial message may end up in `/tf_static` (latched), or the TF buffer retains it. When slam_toolbox subsequently publishes live `map → odom` transforms with current timestamps, TF2 sees both — the old startup message and the new current ones. When a new subscriber (RViz2, another SLAM instance) connects, the startup message is replayed from the latched buffer, arriving with a timestamp from minutes in the past. TF2 logs `TF_OLD_DATA` for every such replay.

**Status:** This warning is cosmetic and does not prevent SLAM from functioning once the scan beam count issue (Issue 3) is fixed. The `last_stamp` monotonic guard in `pseudo_odom_node` prevents a separate (related) source of `TF_OLD_DATA` from pseudo_odom's own publishing.

**Workaround:** The warnings stop once slam_toolbox starts processing scans normally and the TF buffer ages out the stale startup message. The `tf_buffer_duration: 30.0` setting means the stale message is retained in the buffer for 30 seconds after slam_toolbox starts.

---

### Issue 7 — Zombie Processes After Ctrl+C

**Symptom:** After killing the launch with Ctrl+C, `TF_OLD_DATA` warnings continue on the next run and node discovery shows old nodes still running.

**Cause:** `ExecuteProcess` in ROS2 launch (used for `sim_main.py`) does not always receive SIGTERM cleanly in WSL2. The Python process may survive. Additionally, `ros2 daemon stop && ros2 daemon start` stops the discovery daemon but does **not** kill running ROS2 node processes.

**Fix:** After Ctrl+C, run:

```bash
pkill -f "sim_main.py"
pkill -f "async_slam_toolbox_node"
```

Or to kill all Python/ROS processes in the workspace:

```bash
pkill -f "python3.*sim"
```

Then relaunch.

---

## 5. Step-by-Step Run Instructions

### One-Time Setup (Fresh Machine)

```bash
# 1. Install ROS2 Humble (follow official docs, then:)
source /opt/ros/humble/setup.bash

# 2. Install system dependencies
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-ackermann-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard

# 3. Install Python dependencies
pip3 install numpy scipy Pillow transforms3d pyyaml \
  "gym==0.19.0" numba "pyglet==1.5.27" pyopengl

# 4. Clone the simulator
mkdir -p ~/sim/src/f1tenth_simulator
cd ~/sim/src/f1tenth_simulator
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
git clone https://github.com/f1tenth/f1tenth_gym.git
pip3 install -e f1tenth_gym/

# 5. Apply modifications documented in Section 3
#    (gym_bridge.py: publish_tf param + angle_inc fix)
#    (setup.py: pseudo_odom_node entry point)
#    (add pseudo_odom_node.py, mapper_params.yaml, slam_mapping.launch.py, slam_rviz.rviz)

# 6. Build
cd ~/sim
colcon build --symlink-install
source install/setup.bash
```

### Every Launch

Open two terminals. Both must have the workspace sourced.

**Terminal 1 — Launch the full SLAM stack:**

```bash
source /opt/ros/humble/setup.bash
source ~/sim/install/setup.bash
ros2 launch f1tenth_gym_ros slam_mapping.launch.py
```

To launch without RViz2 (headless / SSH):

```bash
ros2 launch f1tenth_gym_ros slam_mapping.launch.py rviz:=false
```

**What to watch for:**
- `gym_bridge` should print the map loading message within 2–3 seconds.
- No `LaserRangeScan contains 1080 range readings, expected 1081` messages.
- `ros2 topic hz /map` (in Terminal 2) should start printing ~0.5 Hz within 10 seconds of the car beginning to move.
- In RViz2, the grey map should begin to grow within 5–10 seconds.

**Terminal 2 — Verify SLAM is running:**

```bash
source /opt/ros/humble/setup.bash
source ~/sim/install/setup.bash

# Confirm scan rate
ros2 topic hz /ego_racecar/scan

# Confirm odom rate (~50 Hz means pseudo_odom is running)
ros2 topic hz /odom

# Confirm map is being built (~0.5 Hz)
ros2 topic hz /map

# View full TF tree (writes frames.gv and frames.pdf to current directory)
ros2 run tf2_tools view_frames
```

### Save the Map (While Stack is Running)

In a **third terminal:**

```bash
source /opt/ros/humble/setup.bash
source ~/sim/install/setup.bash
bash ~/sim/save_map.sh
```

Expected output:

```
==> Serializing slam_toolbox pose graph (reloadable session)...
response: slam_toolbox.srv.SerializePoseGraph_Response(result=0)

==> Saving viewable occupancy grid (subscribes /map with Transient Local QoS)...
[INFO] [map_pgm_saver]: Waiting for /map ...
[INFO] [map_pgm_saver]: Saved /home/<user>/f1tenth_maps/track_map.pgm
[INFO] [map_pgm_saver]: Saved /home/<user>/f1tenth_maps/track_map.yaml

Files saved to /home/<user>/f1tenth_maps:
  Reloadable session : track_map.data + track_map.posegraph
  Viewable map       : track_map.pgm + track_map.yaml
```

### Convert and View the Map on Windows via WSL

```bash
# Option A — Python (no extra install required)
python3 -c "
from PIL import Image
Image.open('$HOME/f1tenth_maps/track_map.pgm').save('$HOME/f1tenth_maps/track_map.png')
"

# Option B — ImageMagick
convert ~/f1tenth_maps/track_map.pgm ~/f1tenth_maps/track_map.png
```

Then open in Windows Explorer at:

```
\\wsl$\Ubuntu\home\<username>\f1tenth_maps\track_map.png
```

### Shut Down Cleanly

```bash
# Terminal 1: press Ctrl+C

# Then kill any survivors (especially sim_main.py via ExecuteProcess)
pkill -f "sim_main.py"
pkill -f "async_slam_toolbox_node"
```

---

## 6. Key ROS2 Topics and TF Tree

### Topics

| Topic | Type | Publisher | Frequency |
|---|---|---|---|
| `/ego_racecar/scan` | `sensor_msgs/LaserScan` | `gym_bridge` | 250 Hz |
| `/ego_racecar/odom` | `nav_msgs/Odometry` | `gym_bridge` | 250 Hz |
| `/ego_racecar/drive` | `ackermann_msgs/AckermannDriveStamped` | `sim_main.py` (wall-follower) | ~250 Hz |
| `/odom` | `nav_msgs/Odometry` | `pseudo_odom_node` | 50 Hz |
| `/map` | `nav_msgs/OccupancyGrid` | `slam_toolbox` | ~0.5 Hz |
| `/tf` | `tf2_msgs/TFMessage` | `gym_bridge`, `pseudo_odom_node`, `slam_toolbox` | varies |
| `/tf_static` | `tf2_msgs/TFMessage` | (startup artifacts, see Known Issues) | once |

### TF Tree

```
map
 └── odom                          ← published by slam_toolbox (50 Hz, /tf)
      └── ego_racecar/base_link    ← published by pseudo_odom_node (50 Hz, /tf)
           ├── ego_racecar/laser   ← published by gym_bridge (250 Hz, /tf)
           ├── ego_racecar/front_left_hinge
           │    └── ego_racecar/front_left_wheel   ← gym_bridge (250 Hz)
           └── ego_racecar/front_right_hinge
                └── ego_racecar/front_right_wheel  ← gym_bridge (250 Hz)
```

Note: `ego_racecar/odom` (the Odometry topic published by gym_bridge with `frame_id='map'`) is **not** a TF frame. It is a topic used for visualization only. The actual TF for odometry comes from `pseudo_odom_node`.

### Health Check Commands

```bash
# TF is fully connected (no "frame does not exist" errors)
ros2 run tf2_ros tf2_echo map ego_racecar/laser

# slam_toolbox can look up odom → base_link
ros2 run tf2_ros tf2_echo odom ego_racecar/base_link

# See full tree topology
ros2 run tf2_tools view_frames
```

---

## 7. What Was Intentionally Left Out and Why

### No Wheel Encoders

The F1Tenth 1:10 platform does not have wheel encoders in the simulator model. Real-world F1Tenth cars also commonly omit them or have low-resolution encoders. Dead-reckoning from commanded speed and steering is an acceptable substitute inside the simulator because the gym's physics engine executes commands faithfully (no slip, no latency). On real hardware, wheel slip, actuator delays, and motor nonlinearity would degrade pseudo-odometry significantly; an IMU or encoder fusion would be needed.

### No IMU

Adding a simulated IMU would improve heading estimates significantly, especially during fast cornering where pure bicycle model integration accumulates error. slam_toolbox can fuse IMU data through its `imu_topic` parameter if a compatible `sensor_msgs/Imu` publisher is available. This was omitted because the simulator doesn't publish IMU data by default and the bicycle model is sufficient for slow mapping runs.

### No Localization-Only Mode

This project only covers map building. The next phase — localization on a previously built map — requires switching slam_toolbox from `mode: mapping` to `mode: localization` and providing the saved `.data`/`.posegraph` files. This was intentionally deferred.

### No Racing or Waypoint Following

The wall-follower is the simplest controller that generates enough robot motion for SLAM. A racing-line planner, pure pursuit controller, or MPC would replace it in a racing-capable stack. These depend on having a completed map and localization, which is why they come after mapping.

---

## 8. Next Steps

### Phase 2 — Localization Only (AMCL or slam_toolbox Localization Mode)

Change `mapper_params.yaml`:

```yaml
mode: localization
map_file_name: /home/<user>/f1tenth_maps/track_map
map_start_at_dock: true
```

This tells slam_toolbox to load the saved pose graph and localize the robot within it rather than building a new map. No other code changes are needed.

For AMCL-based localization (standard nav2 approach):

```bash
ros2 launch nav2_bringup localization_launch.py \
  map:=/home/<user>/f1tenth_maps/track_map.yaml \
  use_sim_time:=false
```

Pair this with `robot_localization` EKF to fuse pseudo-odometry with AMCL pose estimates for smoother state estimation:

```bash
sudo apt install ros-humble-robot-localization
```

### Phase 3 — Waypoint Following

Once localization is working, record a racing line by driving the track once and logging `/ego_racecar/odom` poses, then use a pure pursuit or Stanley controller to follow those waypoints. The `f1tenth_gym_ros` repository includes example waypoint follower implementations.

### Phase 4 — Replace Pseudo-Odometry with Real Odometry

On real hardware, wheel encoder ticks should replace commanded-speed integration. The `pseudo_odom_node.py` entry point in `setup.py` makes it easy to swap in a new odometry implementation: write a new node that subscribes to encoder topics, publish the same `/odom` + TF, and update the launch file to use it instead.

### Improving Map Quality

- **Add IMU:** Fuse with pseudo-odometry via `robot_localization` EKF, passing the fused pose as the odometry source instead of raw bicycle model output.
- **Slow down further:** `max_speed: 0.3` improves scan overlap at corners.
- **More loop closures:** Lower `loop_match_minimum_response_coarse` to `0.2` to accept more loop closure candidates (at the cost of possible false matches on symmetric tracks).
- **Higher resolution:** `resolution: 0.025` (2.5 cm) for finer wall detail, at the cost of larger map files and slower loop closure search.
