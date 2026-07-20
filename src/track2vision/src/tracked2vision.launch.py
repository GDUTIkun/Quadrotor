from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CARTOGRAPHER_START_DELAY_SECONDS = 3.0


def generate_launch_description():
    # 获取包的路径
    lslidar_driver_launch_path = PathJoinSubstitution([FindPackageShare("lslidar_driver"), "launch", "lsm10_uart_launch.py"])
    # 实时建图版：
    cartographer_ros_launch_path = PathJoinSubstitution([FindPackageShare("carto"), "launch", "my_laser_with_imu.launch.py"])
    # pbstream localization 版：
    # cartographer_ros_launch_path = PathJoinSubstitution([FindPackageShare("carto"), "launch", "my_backpack_2d_localization.launch.py"])
    mavros_launch_path = PathJoinSubstitution([FindPackageShare("mavros"), "launch", "px4.launch"])

    return LaunchDescription([
        DeclareLaunchArgument(
            "load_state_filename",
            default_value="/home/pi5/flight_ori_ws/src/carto/map/flight_area.pbstream",
            description="Path to the saved Cartographer .pbstream map for localization.",
        ),

        # 底层数据源先启动；依赖数据和 TF 的节点会在自身逻辑里等待就绪。
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lslidar_driver_launch_path),
            launch_arguments={}.items()
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch_path),
            launch_arguments={}.items()
        ),

        Node(
            package='track2vision',
            executable='imu_covariance_filter',
            name='imu_covariance_filter',
            output='screen'
        ),

        # localization 版 cartographer launch 不发布 base_link -> laser，
        # 但 /scan 的 frame_id 是 laser，Cartographer 需要这个静态 TF。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_static_tf',
            arguments=[
                '0', '0', '0.0448',
                '0', '0', '0', '1',
                'base_link', 'laser'
            ],
            output='screen'
        ),

        TimerAction(
            period=CARTOGRAPHER_START_DELAY_SECONDS,
            actions=[
                # 实时建图版不需要 launch_arguments。
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cartographer_ros_launch_path),
                    launch_arguments={}.items()
                )
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource(cartographer_ros_launch_path),
                #     launch_arguments={
                #         "load_state_filename": LaunchConfiguration("load_state_filename"),
                #     }.items()
                # )
            ]
        ),

        Node(
            package='track2vision',
            executable='cartographer_laser_transfer',
            name='cartographer_laser_transfer',
            output='screen'
        ),

        # #监听offboard完成后，关闭其他节点
        # Shutdown(
        #     condition=launch.conditions.LaunchConfigurationEquals('offboard_node', 'done')
        # )
    ])
