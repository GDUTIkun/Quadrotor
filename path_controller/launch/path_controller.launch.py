from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='car_carto_map'),
        DeclareLaunchArgument('base_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('target_speed_m_s', default_value='0.05'),
        DeclareLaunchArgument('v_max_m_s', default_value='0.05'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.5'),
        DeclareLaunchArgument('heading_tolerance_rad', default_value='0.4'),
        DeclareLaunchArgument('lookahead_distance_m', default_value='0.20'),
        Node(
            package='path_controller',
            executable='path_controller_node',
            name='path_controller_node',
            output='screen',
            parameters=[{
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'target_speed_m_s': LaunchConfiguration('target_speed_m_s'),
                'v_max_m_s': LaunchConfiguration('v_max_m_s'),
                'w_max_rad_s': LaunchConfiguration('w_max_rad_s'),
                'heading_tolerance_rad': LaunchConfiguration('heading_tolerance_rad'),
                'lookahead_distance_m': LaunchConfiguration('lookahead_distance_m'),
            }],
        ),
    ])
