from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vision_start_delay', default_value='15.0'),
        DeclareLaunchArgument('offboard_start_delay', default_value='30.0'),
        Node(
            package='offboard_control',
            executable='key_mission_launcher.py',
            name='key_mission_launcher',
            output='screen',
            parameters=[{
                'vision_start_delay': ParameterValue(
                    LaunchConfiguration('vision_start_delay'), value_type=float),
                'offboard_start_delay': ParameterValue(
                    LaunchConfiguration('offboard_start_delay'), value_type=float),
            }],
        ),
    ])
