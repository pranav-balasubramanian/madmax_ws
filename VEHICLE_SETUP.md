# F1Tenth Real Hardware — SLAM Mapping Setup Guide

This document covers everything needed to run the SLAM mapping stack on the physical F1Tenth car.
The implementation is based on the same approach documented in `HANDOFF.md` (simulator), with all
simulator-specific components replaced by real-hardware equivalents.

---

## Architecture Overview

```
urg_node (LiDAR driver)
  └─► /scan  (LaserScan, ~40 Hz)  ──────────────────────► slam_toolbox
                                                               │
wall_follower ◄── /scan                                        │
  └─► /drive (AckermannDriveStamped) ──────────────────────── │
              │                                                │
              └─► pseudo_odom_node                             │
                      │                                        │
                      ├─► /odom  (nav_msgs/Odometry, 50 Hz)   │
                      └─► TF: odom → base_link (50 Hz)        │

static_transform_publisher:
  └─► TF: base_link → laser  (once at startup)

slam_toolbox publishes:
  /map  (nav_msgs/OccupancyGrid, ~0.5 Hz while mapping)
  TF: map → odom  (50 Hz once scans are processing)
```

**TF tree:**
```
map
 └── odom                ← slam_toolbox
      └── base_link      ← pseudo_odom_node
           └── laser     ← static_transform_publisher
```

---

## Prerequisites

### ROS2 Packages

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-ackermann-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  ros-humble-urg-node
```

### Python Dependencies

```bash
pip3 install numpy scipy
```

### Build the Package

After cloning or pulling changes, rebuild so the new entry points are registered:

```bash
cd ~/f1tenth_ws   # or wherever this workspace lives on the car
colcon build --symlink-install
source install/setup.bash
```

> **Important:** Entry point changes (`setup.py`) always require a full `colcon build`.
> `--symlink-install` does not update entry points on its own.

---

## Step 1: Identify LiDAR Topic and Frame

Start the sensor driver and confirm what it publishes:

```bash
# In one terminal — start just the LiDAR
ros2 launch f1tenth_control sensors_launch.py start_cam:=false start_lidar:=true

# In another terminal — verify
ros2 topic list | grep scan          # should show /scan
ros2 topic hz /scan                  # should show ~40 Hz
ros2 topic echo /scan --once | head -10   # check frame_id field
```

Expected output from `echo`:
```
header:
  frame_id: laser
angle_min: -3.14...
angle_max: 3.14...
```

If the frame is different (e.g., `lidar`, `hokuyo`), pass it as a launch argument:

```bash
ros2 launch f1tenth_control slam_mapping.launch.py lidar_frame:=lidar
```

---

## Step 2: Verify the Drive Topic

The VESC driver must be running and accepting `/drive` commands before starting the SLAM launch.
Check it is active:

```bash
ros2 topic list | grep drive    # should show /drive
ros2 topic info /drive          # should show a subscriber from the VESC node
```

If `/drive` has no subscribers the motor controller is not running. Start the VESC stack first.

---

## Step 3: Start the SLAM Stack

**Terminal 1 — sensors (LiDAR only, camera optional):**

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_control sensors_launch.py start_cam:=false start_lidar:=true
```

**Terminal 2 — SLAM mapping stack:**

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash
ros2 launch f1tenth_control slam_mapping.launch.py
```

Available launch arguments (all optional):

| Argument | Default | Description |
|---|---|---|
| `scan_topic` | `/scan` | LiDAR topic name |
| `lidar_frame` | `laser` | LiDAR TF frame ID |
| `laser_x_offset` | `0.27` | LiDAR forward offset from base_link (meters) |
| `wheelbase` | `0.3` | Bicycle model wheelbase (meters) |
| `speed_scale` | `1.0` | Commanded-speed multiplier for odometry calibration |
| `max_speed` | `0.75` | Wall-follower speed cap (m/s) |
| `rviz` | `true` | Set `false` to suppress RViz2 |

Example with non-default arguments:

```bash
ros2 launch f1tenth_control slam_mapping.launch.py \
  lidar_frame:=lidar \
  laser_x_offset:=0.30 \
  wheelbase:=0.325 \
  speed_scale:=0.9 \
  max_speed:=0.5 \
  rviz:=false
```

---

## Step 4: Enable the Deadman's Switch

The wall-follower will NOT move the car on startup. You must explicitly enable it after confirming
the SLAM stack is healthy (see Step 5 below).

```bash
# In a new terminal — enable driving
ros2 param set /wall_follower enabled true
```

To stop the car at any time:

```bash
ros2 param set /wall_follower enabled false
```

Or press `Ctrl+C` in Terminal 2 — the node publishes a zero-speed command on shutdown.

---

## Step 5: Verify Everything Is Healthy Before Enabling

Run these checks before enabling the deadman's switch:

```bash
# LiDAR is publishing
ros2 topic hz /scan

# Pseudo-odometry is running (~50 Hz)
ros2 topic hz /odom

# SLAM map is being built (~0.5 Hz — may take 10–20 s to appear)
ros2 topic hz /map

# TF tree is fully connected (no "frame does not exist" errors)
ros2 run tf2_ros tf2_echo map laser

# Full TF tree topology
ros2 run tf2_tools view_frames
```

Expected `view_frames` output:
```
map → odom → base_link → laser
```

If `map` is not in the tree yet, slam_toolbox hasn't processed enough scans. Wait a few seconds
and retry. The car must be stationary (deadman off) while you verify.

---

## Step 6: Calibrate Speed Scaling (Optional but Recommended)

The bicycle model uses commanded speed as its motion estimate. On real hardware, commanded speed
does not always equal actual speed due to motor dynamics and wheel slip.

**Procedure:**

1. Mark a 2-meter straight line on the floor.
2. Start the stack with `enabled:=false` and default `speed_scale:=1.0`.
3. Enable the car (`ros2 param set /wall_follower enabled true`) and drive straight for ~2 s.
4. Disable (`ros2 param set /wall_follower enabled false`).
5. Check `ros2 topic echo /odom --once` — read `pose.pose.position.x`.
6. Measure actual distance traveled with a tape measure.
7. `speed_scale = actual_distance / odometry_reported_distance`.

Example: if odom says 2.1 m but car moved 1.8 m → `speed_scale = 1.8 / 2.1 ≈ 0.857`.

Apply in the launch:

```bash
ros2 launch f1tenth_control slam_mapping.launch.py speed_scale:=0.857
```

---

## Step 7: Save the Map

While the SLAM stack is running (Terminal 2), open a third terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/f1tenth_ws/install/setup.bash
bash ~/f1tenth_ws/save_map.sh
```

Expected output:

```
==> Serializing slam_toolbox pose graph (reloadable session)...
response: slam_toolbox.srv.SerializePoseGraph_Response(result=0)

==> Saving viewable occupancy grid ...
[INFO] [map_pgm_saver]: Waiting for /map ...
[INFO] [map_pgm_saver]: Saved /home/<user>/f1tenth_maps/track_map.pgm
[INFO] [map_pgm_saver]: Saved /home/<user>/f1tenth_maps/track_map.yaml

Files saved to /home/<user>/f1tenth_maps:
  Reloadable session : track_map.data + track_map.posegraph
  Viewable map       : track_map.pgm + track_map.yaml
```

If the pose graph serialization returns `result=1` (failure), slam_toolbox has not yet published
a map. Drive for at least one full circuit and wait for `/map` to appear before saving.

### View the Map

```bash
# Convert PGM → PNG for easy viewing
python3 -c "
from PIL import Image
Image.open('$HOME/f1tenth_maps/track_map.pgm').save('$HOME/f1tenth_maps/track_map.png')
"
```

---

## Step 8: Shut Down Cleanly

```bash
# Terminal 2: Ctrl+C
# The wall_follower node sends a zero-speed stop command on shutdown.

# If any nodes survive (check with `ros2 node list`):
pkill -f "wall_follower"
pkill -f "pseudo_odom_node"
pkill -f "async_slam_toolbox_node"
```

---

## Troubleshooting

### `/map` never appears (ros2 topic hz /map shows 0 Hz)

- Confirm `/scan` is publishing: `ros2 topic hz /scan`
- Check slam_toolbox logs for `LaserRangeScan contains N range readings, expected M`. If this
  appears, the `angle_inc` in the LiDAR driver config does not match what slam_toolbox expects.
  Try adjusting the `scan_topic` or checking `sensors.yaml` for `angle_min`/`angle_max`.
- Confirm the TF chain is complete: `ros2 run tf2_ros tf2_echo map laser`

### `TF_OLD_DATA` warnings at startup

Cosmetic — slam_toolbox publishes an identity `map → odom` at startup that gets replayed to late
subscribers. These stop once slam_toolbox begins processing scans normally.

### Car doesn't move after `ros2 param set /wall_follower enabled true`

- Confirm the VESC driver is running and `/drive` has a subscriber.
- Confirm `/scan` is publishing (wall_follower only drives while scans arrive).
- Check `ros2 node list` to ensure `wall_follower` is running.

### Map looks blurry or has tears at corners

- Reduce `max_speed` in the launch (try `0.4` or `0.3`).
- Check `speed_scale` calibration — an overestimated odometry causes SLAM to "stretch" the map.
- Confirm `laser_x_offset` is correct. A wrong offset shifts scan points relative to base_link.

### Odometry drifts significantly

- Calibrate `speed_scale` (see Step 6).
- Tune `wheelbase` to match the actual car measurement (measure front axle to rear axle).
- The bicycle model assumes no wheel slip — at high speeds or sharp turns this breaks down.
  Keep `max_speed` below 0.5 m/s for the best map quality.

---

## File Reference

| File | Purpose |
|---|---|
| `src/f1tenth_control/f1tenth_control/wall_follower.py` | PID wall-follower with e-stop and deadman's switch |
| `src/f1tenth_control/f1tenth_control/pseudo_odom_node.py` | Bicycle model dead-reckoning odometry |
| `src/f1tenth_control/config/mapper_params.yaml` | slam_toolbox tuning for real hardware |
| `src/f1tenth_control/launch/slam_mapping.launch.py` | Full SLAM stack launch |
| `src/f1tenth_control/rviz/slam_mapping.rviz` | RViz2 configuration |
| `save_map.sh` | Save SLAM session + PGM image |
| `save_pgm.py` | Map topic subscriber / PGM writer (Transient Local QoS) |
