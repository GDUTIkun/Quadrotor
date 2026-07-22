from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baud = LaunchConfiguration("serial_baud")
    path_color = LaunchConfiguration("path_color")
    log_serial_rx = LaunchConfiguration("log_serial_rx")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyS7"),
            DeclareLaunchArgument("serial_baud", default_value="115200"),
            DeclareLaunchArgument("path_color", default_value="2016"),
            DeclareLaunchArgument("log_serial_rx", default_value="true"),
            Node(
                package="quadrotor_ground_station",
                executable="best_path_planner_node",
                name="best_path_planner",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "serial_baud": serial_baud,
                        "serial_poll_ms": 50,
                        "path_color": path_color,
                        "log_serial_rx": log_serial_rx,
                        "frame_id": "map",
                        "cell_size": 1.0,
                        "origin_x": 0.0,
                        "origin_y": 0.0,
                    }
                ],
            ),
        ]
    )
