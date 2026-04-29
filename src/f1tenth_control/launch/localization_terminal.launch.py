"""AMCL localization stack with no RViz, no EKF, plus terminal viewer.

Wraps localization.launch.py with rviz:=false and ekf:=false, then adds
the terminal_map_viewer node so a saved map is rendered to the terminal
with the AMCL pose drawn as a red arrow — no window forwarding required.

Usage:
  ros2 launch f1tenth_control localization_terminal.launch.py
  ros2 launch f1tenth_control localization_terminal.launch.py \\
      map:=~/f1tenth_mapper/track_map.yaml refresh_hz:=4.0

Note: the localization nodes' log spam will interleave with the viewer's
ANSI redraw if everything runs in one terminal. For a clean display,
run the viewer in its own terminal instead:
  Terminal A:  ros2 launch f1tenth_control localization.launch.py rviz:=false ekf:=false
  Terminal B:  ros2 run f1tenth_control terminal_map_viewer \\
                   --ros-args -p map_yaml:=~/f1tenth_mapper/track_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('f1tenth_control')

    default_map = os.path.expanduser('~/f1tenth_mapper/track_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Absolute path to the saved map YAML file'
    )
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic', default_value='/amcl_pose',
        description='Pose topic the terminal viewer subscribes to'
    )
    refresh_arg = DeclareLaunchArgument(
        'refresh_hz', default_value='5.0',
        description='Terminal viewer redraw rate (Hz)'
    )
    wheelbase_arg = DeclareLaunchArgument(
        'wheelbase', default_value='0.3',
        description='Vehicle wheelbase for pseudo-odometry'
    )
    speed_scale_arg = DeclareLaunchArgument(
        'speed_scale', default_value='1.0',
        description='Pseudo-odometry speed calibration multiplier'
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'wheelbase': LaunchConfiguration('wheelbase'),
            'speed_scale': LaunchConfiguration('speed_scale'),
            'rviz': 'false',
            'ekf': 'false',
        }.items(),
    )

    viewer = Node(
        package='f1tenth_control',
        executable='terminal_map_viewer',
        name='terminal_map_viewer',
        parameters=[{
            'map_yaml': LaunchConfiguration('map'),
            'pose_topic': LaunchConfiguration('pose_topic'),
            'refresh_hz': LaunchConfiguration('refresh_hz'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        map_arg, pose_topic_arg, refresh_arg, wheelbase_arg, speed_scale_arg,
        localization, viewer,
    ])
