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
        DeclareLaunchArgument('base_frame_id', default_value='car_base_link'),
        DeclareLaunchArgument('odom_frame_id', default_value='car_odom'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('imu_topic', default_value='/car/imu/data_valid'),
        DeclareLaunchArgument('status_topic', default_value='/car/stm/status'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/car/odom/wheel'),
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
                'base_frame_id': LaunchConfiguration('base_frame_id'),
                'odom_frame_id': LaunchConfiguration('odom_frame_id'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
                'wheel_odom_topic': LaunchConfiguration('wheel_odom_topic'),
                'publish_wheel_odom': LaunchConfiguration('publish_wheel_odom'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }],
        ),
    ])
