from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    timeout_ms = LaunchConfiguration("timeout_ms")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyS7"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("timeout_ms", default_value="500"),
            Node(
                package="quadrotor_ground_station",
                executable="serial_loopback_test_node",
                name="serial_loopback_test_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baud_rate": baud_rate,
                        "timeout_ms": timeout_ms,
                    }
                ],
            ),
        ]
    )
