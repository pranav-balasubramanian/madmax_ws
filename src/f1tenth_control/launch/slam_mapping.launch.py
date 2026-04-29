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
        'scan_topic_for_slam', default_value='/scan_for_slam',
        description='Gated scan topic that slam_toolbox subscribes to '
                    '(supervisor opens the gate after ramp-down)'
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
        'max_speed', default_value='2.5',
        description='Hard cap for wall_follower target_speed; must exceed boost_speed'
    ))
    ld.add_action(DeclareLaunchArgument(
        'boost_speed', default_value='2.2',
        description='Initial higher speed used to overcome static torque'
    ))
    ld.add_action(DeclareLaunchArgument(
        'mapping_speed', default_value='1.5',
        description='Steady-state speed once SLAM begins collecting'
    ))
    ld.add_action(DeclareLaunchArgument(
        'boost_duration', default_value='0.75',
        description='Seconds to hold boost_speed before ramp-down'
    ))
    ld.add_action(DeclareLaunchArgument(
        'ramp_duration', default_value='1.5',
        description='Seconds to interpolate from boost_speed down to mapping_speed'
    ))
    ld.add_action(DeclareLaunchArgument(
        'min_lap_time', default_value='20.0',
        description='Earliest time after mapping starts that a lap may be detected'
    ))
    ld.add_action(DeclareLaunchArgument(
        'loop_distance_threshold', default_value='0.5',
        description='Distance (m) from start pose that counts as a closed loop'
    ))
    ld.add_action(DeclareLaunchArgument(
        'map_output_dir', default_value='~/f1tenth_mapper',
        description='Directory where track_map.pgm/.yaml are written on lap completion'
    ))
    ld.add_action(DeclareLaunchArgument(
        'map_name', default_value='track_map',
        description='Base filename (without extension) used for the saved map'
    ))
    ld.add_action(DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to false to suppress RViz2 (headless / SSH sessions)'
    ))

    mapper_params = os.path.join(pkg, 'config', 'mapper_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'slam_mapping.rviz')

    # ── Wall follower ─────────────────────────────────────────────────────────
    # Steers via PID; speed comes from the supervisor over /wall_follower/target_speed.
    # Y-button deadman remains in effect — releasing Y still stops the car.
    wall_follower_node = Node(
        package='f1tenth_control',
        executable='wall_follower',
        name='wall_follower',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
            'drive_topic': '/drive',
            'max_speed': LaunchConfiguration('max_speed'),
            'target_speed_topic': '/wall_follower/target_speed',
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

    # ── Mapping supervisor ────────────────────────────────────────────────────
    # Runs the BOOST → RAMP → SLAM → LAP → SAVE state machine, including the
    # /scan -> /scan_for_slam gate.
    mapping_supervisor_node = Node(
        package='f1tenth_control',
        executable='mapping_supervisor_node',
        name='mapping_supervisor_node',
        parameters=[{
            'boost_speed': LaunchConfiguration('boost_speed'),
            'mapping_speed': LaunchConfiguration('mapping_speed'),
            'boost_duration': LaunchConfiguration('boost_duration'),
            'ramp_duration': LaunchConfiguration('ramp_duration'),
            'min_lap_time': LaunchConfiguration('min_lap_time'),
            'loop_distance_threshold': LaunchConfiguration('loop_distance_threshold'),
            'drive_topic': '/drive',
            'odom_topic': '/odom',
            'scan_topic_in': LaunchConfiguration('scan_topic'),
            'scan_topic_out': LaunchConfiguration('scan_topic_for_slam'),
            'target_speed_topic': '/wall_follower/target_speed',
            'map_output_dir': LaunchConfiguration('map_output_dir'),
            'map_name': LaunchConfiguration('map_name'),
        }],
        output='screen',
    )

    # ── slam_toolbox async ────────────────────────────────────────────────────
    # Note: base_link → laser static TF is published by teleop.launch.py.
    # Do not duplicate it here — conflicting transforms break TF lookups.
    # scan_topic is overridden to the gated topic so no scans reach slam_toolbox
    # until the supervisor opens the gate after ramp-down.
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            mapper_params,
            {'scan_topic': LaunchConfiguration('scan_topic_for_slam')},
        ],
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
    ld.add_action(mapping_supervisor_node)
    ld.add_action(slam_node)
    ld.add_action(rviz_node)
    return ld
