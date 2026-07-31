from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    key1_gpio = LaunchConfiguration("key1_gpio")
    key2_gpio = LaunchConfiguration("key2_gpio")
    active_high = LaunchConfiguration("active_high")
    debounce_ms = LaunchConfiguration("debounce_ms")
    poll_hz = LaunchConfiguration("poll_hz")
    state_publish_ms = LaunchConfiguration("state_publish_ms")

    return LaunchDescription(
        [
            DeclareLaunchArgument("key1_gpio", default_value="46"),
            DeclareLaunchArgument("key2_gpio", default_value="47"),
            DeclareLaunchArgument("active_high", default_value="true"),
            DeclareLaunchArgument("debounce_ms", default_value="30"),
            DeclareLaunchArgument("poll_hz", default_value="100.0"),
            DeclareLaunchArgument("state_publish_ms", default_value="200"),
            Node(
                package="gpio_keys",
                executable="gpio_keys_node.py",
                name="gpio_keys_node",
                output="screen",
                parameters=[
                    {
                        "key1_gpio": key1_gpio,
                        "key2_gpio": key2_gpio,
                        "active_high": active_high,
                        "debounce_ms": debounce_ms,
                        "poll_hz": poll_hz,
                        "state_publish_ms": state_publish_ms,
                    }
                ],
            ),
        ]
    )
