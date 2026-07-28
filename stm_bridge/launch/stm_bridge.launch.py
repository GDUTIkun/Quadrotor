from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/wheeltec_controller'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('publish_odom_tf', default_value='false'),
        Node(
            package='stm_bridge',
            executable='stm_bridge_node',
            name='stm_bridge_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'publish_odom_tf': LaunchConfiguration('publish_odom_tf'),
            }],
        ),
    ])
