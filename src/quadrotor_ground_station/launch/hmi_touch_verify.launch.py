from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyS7"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            Node(
                package="quadrotor_ground_station",
                executable="hmi_touch_verify_node",
                name="hmi_touch_verify_node",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baud_rate": baud_rate,
                    }
                ],
            ),
        ]
    )
