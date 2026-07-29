from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('radius_m', default_value='0.75'),
        DeclareLaunchArgument('lookahead_distance_m', default_value='0.25'),
        DeclareLaunchArgument('linear_speed_m_s', default_value='0.02'),
        DeclareLaunchArgument('k_w', default_value='0.4'),
        DeclareLaunchArgument('k_w_rate', default_value='0.32'),
        DeclareLaunchArgument('k_i_rate', default_value='0.9'),
        DeclareLaunchArgument('k_d_rate', default_value='0.0'),
        DeclareLaunchArgument('w_error_integral_max', default_value='0.5'),
        DeclareLaunchArgument('w_error_derivative_filter_tau_s', default_value='0.05'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.3'),
        DeclareLaunchArgument('pose_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('odom_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('clockwise', default_value='true'),
        DeclareLaunchArgument('publish_when_idle', default_value='true'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('pose_topic', default_value='/car/pose'),
        DeclareLaunchArgument('odom_topic', default_value='/car/odom/carto'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'command_topic', default_value='/car/circle_angle_tuner/command'),
        DeclareLaunchArgument(
            'speed_topic', default_value='/car/circle_angle_tuner/speed'),
        DeclareLaunchArgument(
            'k_w_topic', default_value='/car/circle_angle_tuner/k_w'),
        DeclareLaunchArgument(
            'lookahead_topic', default_value='/car/circle_angle_tuner/lookahead'),
        DeclareLaunchArgument(
            'status_topic', default_value='/car/circle_angle_tuner/status'),
        Node(
            package='track_runner',
            executable='circle_angle_tuner_node',
            name='circle_angle_tuner_node',
            output='screen',
            parameters=[{
                'radius_m': LaunchConfiguration('radius_m'),
                'lookahead_distance_m': LaunchConfiguration('lookahead_distance_m'),
                'linear_speed_m_s': LaunchConfiguration('linear_speed_m_s'),
                'k_w': LaunchConfiguration('k_w'),
                'k_w_rate': LaunchConfiguration('k_w_rate'),
                'k_i_rate': LaunchConfiguration('k_i_rate'),
                'k_d_rate': LaunchConfiguration('k_d_rate'),
                'w_error_integral_max': LaunchConfiguration('w_error_integral_max'),
                'w_error_derivative_filter_tau_s': LaunchConfiguration(
                    'w_error_derivative_filter_tau_s'),
                'w_max_rad_s': LaunchConfiguration('w_max_rad_s'),
                'pose_timeout_s': LaunchConfiguration('pose_timeout_s'),
                'odom_timeout_s': LaunchConfiguration('odom_timeout_s'),
                'clockwise': LaunchConfiguration('clockwise'),
                'publish_when_idle': LaunchConfiguration('publish_when_idle'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'speed_topic': LaunchConfiguration('speed_topic'),
                'k_w_topic': LaunchConfiguration('k_w_topic'),
                'lookahead_topic': LaunchConfiguration('lookahead_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
            }],
        ),
    ])
