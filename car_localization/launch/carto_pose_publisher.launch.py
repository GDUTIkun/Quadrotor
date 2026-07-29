from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('global_frame_id', default_value='car_carto_map'),
        DeclareLaunchArgument('base_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('output_frame_id', default_value='car_map'),
        DeclareLaunchArgument('pose_topic', default_value='/car/pose'),
        DeclareLaunchArgument('odom_topic', default_value='/car/odom/carto'),
        DeclareLaunchArgument('odom_child_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('yaw_offset_rad', default_value='0.0'),
        DeclareLaunchArgument('publish_odom', default_value='true'),
        DeclareLaunchArgument('velocity_filter_tau_s', default_value='0.25'),
        DeclareLaunchArgument('max_velocity_dt_s', default_value='0.5'),
        Node(
            package='car_localization',
            executable='carto_pose_publisher_node',
            name='car_carto_pose_publisher_node',
            output='screen',
            parameters=[{
                'global_frame_id': LaunchConfiguration('global_frame_id'),
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'output_frame_id': LaunchConfiguration('output_frame_id'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'odom_child_frame_id': LaunchConfiguration('odom_child_frame_id'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                'yaw_offset_rad': LaunchConfiguration('yaw_offset_rad'),
                'publish_odom': LaunchConfiguration('publish_odom'),
                'velocity_filter_tau_s': LaunchConfiguration('velocity_filter_tau_s'),
                'max_velocity_dt_s': LaunchConfiguration('max_velocity_dt_s'),
            }],
        ),
    ])
