from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('straight_length_m', default_value='1.5'),
        DeclareLaunchArgument('radius_m', default_value='0.75'),
        DeclareLaunchArgument('path_spacing_m', default_value='0.02'),
        DeclareLaunchArgument('default_speed_m_s', default_value='0.01'),
        DeclareLaunchArgument('default_laps', default_value='1'),
        DeclareLaunchArgument('lookahead_distance_m', default_value='0.03'),
        DeclareLaunchArgument('waypoint_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('k_w', default_value='0.6'),
        DeclareLaunchArgument('k_w_rate', default_value='1.0'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.3'),
        DeclareLaunchArgument('pose_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('angular_velocity_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('use_angular_velocity_feedback', default_value='true'),
        DeclareLaunchArgument('use_start_pose_as_origin', default_value='true'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('pose_topic', default_value='/car/pose'),
        DeclareLaunchArgument('odom_topic', default_value='/car/odom/carto'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'command_topic', default_value='/car/track_runner/command'),
        DeclareLaunchArgument(
            'speed_topic', default_value='/car/track_runner/speed'),
        DeclareLaunchArgument(
            'laps_topic', default_value='/car/track_runner/laps'),
        DeclareLaunchArgument(
            'status_topic', default_value='/car/track_runner/status'),
        Node(
            package='track_runner',
            executable='track_runner_node',
            name='track_runner_node',
            output='screen',
            parameters=[{
                'straight_length_m': LaunchConfiguration('straight_length_m'),
                'radius_m': LaunchConfiguration('radius_m'),
                'path_spacing_m': LaunchConfiguration('path_spacing_m'),
                'default_speed_m_s': LaunchConfiguration('default_speed_m_s'),
                'default_laps': LaunchConfiguration('default_laps'),
                'lookahead_distance_m': LaunchConfiguration('lookahead_distance_m'),
                'waypoint_tolerance_m': LaunchConfiguration('waypoint_tolerance_m'),
                'goal_tolerance_m': LaunchConfiguration('goal_tolerance_m'),
                'k_w': LaunchConfiguration('k_w'),
                'k_w_rate': LaunchConfiguration('k_w_rate'),
                'w_max_rad_s': LaunchConfiguration('w_max_rad_s'),
                'pose_timeout_s': LaunchConfiguration('pose_timeout_s'),
                'angular_velocity_timeout_s': LaunchConfiguration('angular_velocity_timeout_s'),
                'use_angular_velocity_feedback': LaunchConfiguration(
                    'use_angular_velocity_feedback'),
                'use_start_pose_as_origin': LaunchConfiguration('use_start_pose_as_origin'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'speed_topic': LaunchConfiguration('speed_topic'),
                'laps_topic': LaunchConfiguration('laps_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
            }],
        ),
    ])
