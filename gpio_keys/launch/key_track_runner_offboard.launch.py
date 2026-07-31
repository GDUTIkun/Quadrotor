from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    key1_gpio = LaunchConfiguration("key1_gpio")
    key2_gpio = LaunchConfiguration("key2_gpio")
    active_high = LaunchConfiguration("active_high")
    debounce_ms = LaunchConfiguration("debounce_ms")
    poll_hz = LaunchConfiguration("poll_hz")
    state_publish_ms = LaunchConfiguration("state_publish_ms")
    start_delay_s = LaunchConfiguration("start_delay_s")
    target_args = LaunchConfiguration("target_args")

    return LaunchDescription(
        [
            DeclareLaunchArgument("key1_gpio", default_value="46"),
            DeclareLaunchArgument("key2_gpio", default_value="47"),
            DeclareLaunchArgument("active_high", default_value="true"),
            DeclareLaunchArgument("debounce_ms", default_value="30"),
            DeclareLaunchArgument("poll_hz", default_value="100.0"),
            DeclareLaunchArgument("state_publish_ms", default_value="200"),
            DeclareLaunchArgument("start_delay_s", default_value="3.0"),
            DeclareLaunchArgument("target_args", default_value=""),
            Node(
                package="gpio_keys",
                executable="gpio_keys_node.py",
                name="gpio_keys_node",
                output="screen",
                parameters=[
                    {
                        "key1_gpio": ParameterValue(key1_gpio, value_type=int),
                        "key2_gpio": ParameterValue(key2_gpio, value_type=int),
                        "active_high": ParameterValue(active_high, value_type=bool),
                        "debounce_ms": ParameterValue(debounce_ms, value_type=int),
                        "poll_hz": ParameterValue(poll_hz, value_type=float),
                        "state_publish_ms": ParameterValue(state_publish_ms, value_type=int),
                    }
                ],
            ),
            Node(
                package="gpio_keys",
                executable="key_track_runner_supervisor.py",
                name="key_track_runner_supervisor",
                output="screen",
                parameters=[
                    {
                        "key1_topic": "/gpio_keys/key1",
                        "key2_topic": "/gpio_keys/key2",
                        "target_package": "car_bringup",
                        "target_launch": "track_runner_offboard.launch.py",
                        "target_args": ParameterValue(target_args, value_type=str),
                        "start_delay_s": ParameterValue(start_delay_s, value_type=float),
                        "stop_command_topic": "/car/track_runner/command",
                        "publish_stop_before_kill": True,
                    }
                ],
            ),
        ]
    )
