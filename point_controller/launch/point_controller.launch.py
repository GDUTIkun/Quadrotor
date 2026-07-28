from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('v_max_m_s', default_value='0.25'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.8'),
        Node(
            package='point_controller',
            executable='point_controller_node',
            name='point_controller_node',
            output='screen',
            parameters=[{
                'v_max_m_s': LaunchConfiguration('v_max_m_s'),
                'w_max_rad_s': LaunchConfiguration('w_max_rad_s'),
            }],
        ),
    ])
