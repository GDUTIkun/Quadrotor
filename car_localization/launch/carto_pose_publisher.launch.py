from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='map'),
        DeclareLaunchArgument('base_frame_id', default_value='base_link'),
        DeclareLaunchArgument('output_frame_id', default_value='car_map'),
        DeclareLaunchArgument('pose_topic', default_value='/car/pose'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('yaw_offset_rad', default_value='0.0'),
        Node(
            package='car_localization',
            executable='carto_pose_publisher_node',
            name='carto_pose_publisher_node',
            output='screen',
            parameters=[{
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'yaw_offset_rad': LaunchConfiguration('yaw_offset_rad'),
            }],
        ),
    ])
