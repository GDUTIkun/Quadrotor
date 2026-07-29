from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('baudrate', default_value='576000'),
        DeclareLaunchArgument('diagnostics_rate_hz', default_value='1.0'),
        DeclareLaunchArgument('debug_rx_hex', default_value='false'),
        DeclareLaunchArgument('publish_wheel_odom', default_value='false'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
        Node(
            package='stm_bridge',
            executable='stm_bridge_node',
            name='stm_bridge_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'diagnostics_rate_hz': LaunchConfiguration('diagnostics_rate_hz'),
                'debug_rx_hex': LaunchConfiguration('debug_rx_hex'),
                'publish_wheel_odom': LaunchConfiguration('publish_wheel_odom'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }],
        ),
    ])
