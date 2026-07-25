from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="nuedc25_ground_station",
                executable="ground_station_gui.py",
                name="ground_station_gui",
                output="screen",
            ),
            Node(
                package="nuedc25_ground_station",
                executable="ground_station_path_node",
                name="ground_station_path_node",
                output="screen",
                parameters=[
                    {
                        "input_topic": "patrol_path_grid",
                        "output_topic": "ground_station_flight_path",
                        "frame_id": "map",
                        "origin_x": 0.0,
                        "origin_y": 0.0,
                        "flight_z": 0.8,
                        "cell_size": 0.4,
                        "qos_depth": 10,
                    }
                ],
            ),
        ]
    )
