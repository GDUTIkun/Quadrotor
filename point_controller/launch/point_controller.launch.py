from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='car_carto_map'),
        DeclareLaunchArgument('base_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('v_max_m_s', default_value='0.25'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.8'),
        Node(
            package='point_controller',
            executable='point_controller_node',
            name='point_controller_node',
            output='screen',
            parameters=[{
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'v_max_m_s': LaunchConfiguration('v_max_m_s'),
                'w_max_rad_s': LaunchConfiguration('w_max_rad_s'),
            }],
        ),
    ])
