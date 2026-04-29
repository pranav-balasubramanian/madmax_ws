"""AMCL localization stack for the real F1Tenth car.

Prerequisites (must already be running):
  - teleop.launch.py        (VESC, ackermann_mux, joy_node, base_link → laser TF)
  - sensors_launch.py       (LiDAR publishing /scan)

What this launch starts:
  - pseudo_odom_node        (publishes /odom and TF odom → base_link)
  - nav2 map_server         (loads the saved map)
  - nav2 amcl               (publishes TF map → odom)
  - nav2 lifecycle_manager  (activates map_server and amcl)
  - robot_localization EKF  (fuses /odom + /amcl_pose → /odometry/filtered)
  - RViz2                   (toggleable)

Usage:
  ros2 launch f1tenth_control localization.launch.py
  ros2 launch f1tenth_control localization.launch.py map:=/path/to/track_map.yaml
  ros2 launch f1tenth_control localization.launch.py rviz:=false ekf:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    ld = LaunchDescription()
    pkg = get_package_share_directory('f1tenth_control')

    default_map = os.path.expanduser('~/f1tenth_maps/track_map.yaml')

    ld.add_action(DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Absolute path to the saved map YAML file'
    ))
    ld.add_action(DeclareLaunchArgument(
        'wheelbase', default_value='0.3',
        description='Vehicle wheelbase for pseudo-odometry bicycle model (meters)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_scale', default_value='1.0',
        description='Multiply commanded speed by this factor before integrating odometry'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to suppress RViz2 (headless / SSH sessions)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'ekf', default_value='true',
        description='Set to false to skip the robot_localization EKF'
    ))

    amcl_params = os.path.join(pkg, 'config', 'amcl_params.yaml')
    ekf_params = os.path.join(pkg, 'config', 'ekf_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'localization.rviz')

    # ── Pseudo odometry ──────────────────────────────────────────────────────
    # Publishes /odom and broadcasts TF odom → base_link, both required by AMCL.
    pseudo_odom_node = Node(
        package='f1tenth_control',
        executable='pseudo_odom_node',
        name='pseudo_odom_node',
        parameters=[{
            'drive_topic': '/drive',
            'wheelbase': LaunchConfiguration('wheelbase'),
            'speed_scale': LaunchConfiguration('speed_scale'),
        }],
        output='screen',
    )

    # ── Map server ────────────────────────────────────────────────────────────
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'topic_name': 'map',
            'frame_id': 'map',
        }],
        output='screen',
    )

    # ── AMCL ─────────────────────────────────────────────────────────────────
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        parameters=[amcl_params],
        output='screen',
    )

    # ── Lifecycle manager ────────────────────────────────────────────────────
    # Configures and activates map_server and amcl in order.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
        output='screen',
    )

    # ── EKF (optional smoothing layer) ───────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_params],
        condition=IfCondition(LaunchConfiguration('ekf')),
        output='screen',
    )

    # ── RViz2 ────────────────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    ld.add_action(pseudo_odom_node)
    ld.add_action(map_server)
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager)
    ld.add_action(ekf_node)
    ld.add_action(rviz_node)
    return ld
