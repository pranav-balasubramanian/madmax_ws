import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    pkg = get_package_share_directory('f1tenth_control')

    # ── Launch arguments ──────────────────────────────────────────────────────
    ld.add_action(DeclareLaunchArgument(
        'scan_topic', default_value='/scan',
        description='LiDAR scan topic published by the sensor driver'
    ))
    ld.add_action(DeclareLaunchArgument(
        'lidar_frame', default_value='laser',
        description='TF frame ID attached to the LiDAR sensor'
    ))
    ld.add_action(DeclareLaunchArgument(
        'laser_x_offset', default_value='0.27',
        description='LiDAR x offset from base_link in meters (positive = forward)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'wheelbase', default_value='0.3',
        description='Vehicle wheelbase in meters for bicycle model odometry'
    ))
    ld.add_action(DeclareLaunchArgument(
        'speed_scale', default_value='1.0',
        description='Multiply commanded speed by this factor before integrating odometry'
    ))
    ld.add_action(DeclareLaunchArgument(
        'max_speed', default_value='0.75',
        description='Maximum drive speed for the wall follower (m/s)'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to suppress RViz2 (headless / SSH sessions)'
    ))

    mapper_params = os.path.join(pkg, 'config', 'mapper_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'slam_mapping.rviz')

    # ── Wall follower ─────────────────────────────────────────────────────────
    # enabled:=False by default — operator must set the parameter before the car moves
    wall_follower_node = Node(
        package='f1tenth_control',
        executable='wall_follower',
        name='wall_follower',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'drive_topic': '/drive',
            'max_speed': LaunchConfiguration('max_speed'),
            'enabled': False,
        }],
        output='screen',
    )

    # ── Pseudo odometry (kinematic bicycle model) ─────────────────────────────
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

    # ── Static TF: base_link → laser ─────────────────────────────────────────
    # gym_bridge published this in the simulator; on real hardware nothing does.
    # The laser is mounted forward of the base at laser_x_offset, same height.
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            LaunchConfiguration('laser_x_offset'), '0', '0',   # x y z
            '0', '0', '0',                                       # roll pitch yaw
            'base_link',
            LaunchConfiguration('lidar_frame'),
        ],
        output='screen',
    )

    # ── slam_toolbox async ────────────────────────────────────────────────────
    # async variant processes scans in a background thread so drive commands
    # and TF publishing continue at full rate during loop closure.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[mapper_params],
        output='screen',
    )

    # ── RViz2 (optional) ─────────────────────────────────────────────────────
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )

    ld.add_action(wall_follower_node)
    ld.add_action(pseudo_odom_node)
    ld.add_action(static_tf_node)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    return ld
