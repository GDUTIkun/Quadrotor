from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("stp23_ros2"), "config", "stp23.yaml"]
    )

    port_name = LaunchConfiguration("port_name")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument("port_name", default_value="auto"),
            DeclareLaunchArgument("frame_id", default_value="lidar_frame"),
            Node(
                package="stp23_ros2",
                executable="stp23_node",
                name="stp23_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "port_name": port_name,
                        "frame_id": frame_id,
                    },
                ],
            ),
        ]
    )
