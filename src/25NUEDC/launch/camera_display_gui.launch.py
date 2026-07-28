from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("DISPLAY", ":0"),
            SetEnvironmentVariable(
                "XAUTHORITY",
                "/run/user/1000/gdm/Xauthority",
            ),
            Node(
                package="nuedc25_ground_station",
                executable="camera_display_gui.py",
                name="camera_display_gui",
                output="screen",
                parameters=[
                    {
                        "image_topic": "/camera/image_raw/compressed",
                    }
                ],
            )
        ]
    )
