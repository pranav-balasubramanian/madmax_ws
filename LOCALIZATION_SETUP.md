# F1Tenth Real Hardware — AMCL Localization Setup Guide

This document covers running AMCL localization on a previously saved SLAM map.
Prerequisite: a usable map at `~/f1tenth_maps/track_map.{pgm,yaml}` produced by
`save_map.sh` (see `VEHICLE_SETUP.md`).

---

## Architecture

### TF Tree
```
map                                    ← AMCL publishes map → odom
 └── odom                              ← pseudo_odom_node publishes odom → base_link
      └── base_link                    ← teleop.launch.py publishes base_link → laser
           └── laser
```

### Node Graph
```
urg_node           ──/scan──────────────────────► AMCL
pseudo_odom_node   ──/odom──────────────────────► AMCL (motion model), EKF
map_server         ──/map───────────────────────► AMCL, RViz

AMCL               ──/amcl_pose─────────────────► EKF, RViz
AMCL               ──TF: map → odom─────────────► all consumers
EKF                ──/odometry/filtered─────────► RViz / downstream nodes
```

---

## Prerequisites

```bash
sudo apt install \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-robot-localization
```

Build the workspace after pulling these new files:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

## Files

| File | Purpose |
|---|---|
| `src/f1tenth_control/config/amcl_params.yaml` | AMCL particle filter and laser model tuning |
| `src/f1tenth_control/config/ekf_params.yaml` | robot_localization EKF — fuses /odom + /amcl_pose |
| `src/f1tenth_control/launch/localization.launch.py` | Full localization stack launch |
| `src/f1tenth_control/rviz/localization.rviz` | RViz config for localization |

---

## Run Sequence

Open **three terminals** on the car. Source ROS and the workspace in each.

**Terminal 1 — VESC + joystick + mux:**
```bash
ros2 launch f1tenth_control teleop.launch.py
```

**Terminal 2 — LiDAR:**
```bash
ros2 launch f1tenth_control sensors_launch.py start_cam:=false start_lidar:=true
```

**Terminal 3 — Localization stack:**
```bash
ros2 launch f1tenth_control localization.launch.py
```

Optional launch args:

| Argument | Default | Description |
|---|---|---|
| `map` | `~/f1tenth_maps/track_map.yaml` | Path to map YAML |
| `wheelbase` | `0.3` | Bicycle model wheelbase |
| `speed_scale` | `1.0` | Pseudo-odometry calibration multiplier |
| `rviz` | `true` | Set `false` to suppress RViz |
| `ekf` | `true` | Set `false` to skip the EKF layer |

Example:
```bash
ros2 launch f1tenth_control localization.launch.py \
  map:=/home/orin/f1tenth_maps/atrium.yaml \
  speed_scale:=0.85 \
  rviz:=false
```

---

## Initialization

After Terminal 3 starts, RViz shows the saved map and a cluster of green particle arrows
spawned at the map origin. The car's actual physical pose is almost certainly not at the origin —
you must seed AMCL with the correct starting pose:

1. In the RViz toolbar, click **2D Pose Estimate** (or press `P`).
2. Click and drag at the car's actual position on the map. The drag direction sets the heading.
3. The particle cloud should snap to a tight cluster at your click point.
4. Drive the car (hold the joystick button used in `teleop.launch.py`). After a few meters of
   motion the cluster tightens further and the red LiDAR points should overlay the black wall
   outlines on the map.

---

## Verification

```bash
# AMCL is publishing a pose estimate
ros2 topic hz /amcl_pose          # ~1–2 Hz when moving, 0 when stationary (normal)

# EKF is producing a smooth pose at 50 Hz
ros2 topic hz /odometry/filtered  # 50 Hz

# Map fired (latched, single message)
ros2 topic echo /map --once | head -5

# TF chain is connected end-to-end
ros2 run tf2_ros tf2_echo map laser
```

The TF echo should print a continuously updating translation. If it errors with "frame does not
exist," one of the upstream broadcasters is missing — re-check the prerequisite launches.

---

## What Good Localization Looks Like

- Red LiDAR points sit directly on the black wall outlines
- Particle cluster spread under 0.5 m and rotates with the car
- `/amcl_pose` covariance values stay below ~0.1 m² for x/y
- The green `/odometry/filtered` axes follow the car smoothly without jumps

## What Broken Localization Looks Like

- Scan floats off the walls or through them
- Particles scatter or jump to a different room/section
- Filtered pose visibly snaps backward when AMCL corrects

---

## Tuning Reference

| Symptom | Parameter to adjust | Direction |
|---|---|---|
| Particles diverge during turns | `alpha1`, `alpha2` in `amcl_params.yaml` | Increase (more rotation noise) |
| Filter doesn't follow forward motion | `alpha3` | Decrease (less translation noise) |
| Localization drifts gradually | `update_min_d`, `update_min_a` | Decrease (update more often) |
| CPU too high | `max_particles` | Decrease |
| Filter unsure / large covariance | `max_particles` | Increase |
| Scan vs map mismatch on coarse maps | `laser_sigma_hit` | Increase |
| Car gets "kidnapped" too aggressively | `recovery_alpha_fast` | Decrease |

---

## Common Failures

### Map appears in RViz but particles never converge
Initial pose is too far from the actual location. Use **2D Pose Estimate** again, more carefully.

### `/amcl_pose` never publishes
AMCL didn't reach the active state. Check the lifecycle_manager output for transition errors.
Most common cause: missing `nav2_amcl` package — install with `apt`.

### TF error: `Could not transform from base_link to laser`
The static TF from `teleop.launch.py` isn't running. Confirm Terminal 1 is still active.

### TF error: `Could not transform from odom to base_link`
`pseudo_odom_node` isn't running or no `/drive` commands have arrived yet to set its initial state.
Check `ros2 node list | grep pseudo_odom_node`.

### Localization works at first, then drifts off the map
Pseudo-odometry is calibrated wrong. Re-run the `speed_scale` calibration procedure from
`VEHICLE_SETUP.md` Step 6 and re-launch with the corrected value.

### `/odometry/filtered` not publishing
EKF is rejecting all inputs. Check `ros2 topic echo /odom --once` and `ros2 topic echo /amcl_pose
--once`. If `/odom` covariance is all zeros, EKF will discard it — this is a known gap of the
current `pseudo_odom_node`. Workaround: launch with `ekf:=false` and consume `/amcl_pose` directly.

---

## Switching to VESC Encoder Odometry (Future Work)

The current stack uses `pseudo_odom_node` (kinematic bicycle model) for the motion source. The
VESC has a built-in motor-pole encoder available via `vesc_to_odom_node`, which is more accurate
under wheel slip. To switch:

1. Set `publish_tf: true` in `config/vesc.yaml` (`vesc_to_odom_node` section)
2. Remove `pseudo_odom_node` from `localization.launch.py`
3. Lower `alpha1`–`alpha4` in `amcl_params.yaml` to ~0.1 (encoder odometry is less noisy)
4. Verify `/odom` is publishing the VESC's output: `ros2 topic info /odom -v`

This change is deferred until VESC odometry is independently verified by pushing the car by hand
and watching `/odom` integrate distance correctly.
