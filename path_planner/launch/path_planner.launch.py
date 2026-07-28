from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_file', default_value=''),
        Node(
            package='path_planner',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'obstacle_file': LaunchConfiguration('obstacle_file'),
            }],
        ),
    ])

