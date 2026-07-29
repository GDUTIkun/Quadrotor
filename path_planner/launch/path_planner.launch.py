from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='car_carto_map'),
        DeclareLaunchArgument('base_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('obstacle_file', default_value=''),
        Node(
            package='path_planner',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'obstacle_file': LaunchConfiguration('obstacle_file'),
            }],
        ),
    ])
