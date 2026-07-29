from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='car_carto_map'),
        DeclareLaunchArgument('base_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('plan_mode', default_value='goal'),
        DeclareLaunchArgument('path_spacing_m', default_value='0.02'),
        DeclareLaunchArgument('test_line_length_m', default_value='0.5'),
        DeclareLaunchArgument('test_arc_radius_m', default_value='0.3'),
        DeclareLaunchArgument('test_arc_angle_rad', default_value='1.57079632679'),
        DeclareLaunchArgument('racetrack_straight_length_m', default_value='1.5'),
        DeclareLaunchArgument('racetrack_radius_m', default_value='0.75'),
        DeclareLaunchArgument('racetrack_turn_right', default_value='true'),
        DeclareLaunchArgument('test_publish_rate_hz', default_value='2.0'),
        DeclareLaunchArgument('test_publish_repeats', default_value='20'),
        Node(
            package='path_planner',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'plan_mode': LaunchConfiguration('plan_mode'),
                'path_spacing_m': LaunchConfiguration('path_spacing_m'),
                'test_line_length_m': LaunchConfiguration('test_line_length_m'),
                'test_arc_radius_m': LaunchConfiguration('test_arc_radius_m'),
                'test_arc_angle_rad': LaunchConfiguration('test_arc_angle_rad'),
                'racetrack_straight_length_m':
                    LaunchConfiguration('racetrack_straight_length_m'),
                'racetrack_radius_m': LaunchConfiguration('racetrack_radius_m'),
                'racetrack_turn_right': LaunchConfiguration('racetrack_turn_right'),
                'test_publish_rate_hz': LaunchConfiguration('test_publish_rate_hz'),
                'test_publish_repeats': LaunchConfiguration('test_publish_repeats'),
            }],
        ),
    ])
