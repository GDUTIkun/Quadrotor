from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('linear_speed_m_s', default_value='0.02'),
        DeclareLaunchArgument('target_w_rad_s', default_value='0.2'),
        DeclareLaunchArgument('k_w_rate', default_value='1.0'),
        DeclareLaunchArgument('k_i_rate', default_value='0.0'),
        DeclareLaunchArgument('w_error_integral_max', default_value='0.5'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.3'),
        DeclareLaunchArgument('odom_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('publish_when_idle', default_value='true'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('odom_topic', default_value='/car/odom/carto'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'command_topic', default_value='/car/angular_rate_tuner/command'),
        DeclareLaunchArgument(
            'speed_topic', default_value='/car/angular_rate_tuner/speed'),
        DeclareLaunchArgument(
            'target_w_topic', default_value='/car/angular_rate_tuner/target_w'),
        DeclareLaunchArgument(
            'k_w_rate_topic', default_value='/car/angular_rate_tuner/k_w_rate'),
        DeclareLaunchArgument(
            'k_i_rate_topic', default_value='/car/angular_rate_tuner/k_i_rate'),
        DeclareLaunchArgument(
            'status_topic', default_value='/car/angular_rate_tuner/status'),
        Node(
            package='track_runner',
            executable='angular_rate_tuner_node',
            name='angular_rate_tuner_node',
            output='screen',
            parameters=[{
                'linear_speed_m_s': LaunchConfiguration('linear_speed_m_s'),
                'target_w_rad_s': LaunchConfiguration('target_w_rad_s'),
                'k_w_rate': LaunchConfiguration('k_w_rate'),
                'k_i_rate': LaunchConfiguration('k_i_rate'),
                'w_error_integral_max': LaunchConfiguration('w_error_integral_max'),
                'w_max_rad_s': LaunchConfiguration('w_max_rad_s'),
                'odom_timeout_s': LaunchConfiguration('odom_timeout_s'),
                'publish_when_idle': LaunchConfiguration('publish_when_idle'),
                'control_rate_hz': LaunchConfiguration('control_rate_hz'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'command_topic': LaunchConfiguration('command_topic'),
                'speed_topic': LaunchConfiguration('speed_topic'),
                'target_w_topic': LaunchConfiguration('target_w_topic'),
                'k_w_rate_topic': LaunchConfiguration('k_w_rate_topic'),
                'k_i_rate_topic': LaunchConfiguration('k_i_rate_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
            }],
        ),
    ])
