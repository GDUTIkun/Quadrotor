from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_controller',
            executable='path_controller_node',
            name='path_controller_node',
            output='screen',
        ),
    ])

