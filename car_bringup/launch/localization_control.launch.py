from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_stm = LaunchConfiguration('start_stm')
    start_carto = LaunchConfiguration('start_carto')
    start_navigation = LaunchConfiguration('start_navigation')
    start_pose_publisher = LaunchConfiguration('start_pose_publisher')

    load_state_filename = LaunchConfiguration('load_state_filename')
    obstacle_file = LaunchConfiguration('obstacle_file')
    pose_topic = LaunchConfiguration('pose_topic')
    pose_publish_rate_hz = LaunchConfiguration('pose_publish_rate_hz')
    pose_yaw_offset_rad = LaunchConfiguration('pose_yaw_offset_rad')

    stm_port = LaunchConfiguration('stm_port')
    stm_baudrate = LaunchConfiguration('stm_baudrate')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')

    stm_node = Node(
        package='stm_bridge',
        executable='stm_bridge_node',
        name='stm_bridge_node',
        output='screen',
        condition=IfCondition(start_stm),
        parameters=[{
            'port': stm_port,
            'baudrate': stm_baudrate,
            'publish_wheel_odom': False,
            'publish_odom_tf': False,
        }],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('lslidar_driver'),
                'launch',
                'lsn10p_launch.py',
            ])
        ),
        condition=IfCondition(start_lidar),
    )

    carto_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carto'),
                'launch',
                'my_backpack_2d_localization.launch.py',
            ])
        ),
        condition=IfCondition(start_carto),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'load_state_filename': load_state_filename,
        }.items(),
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser_tf',
        output='screen',
        arguments=[
            '--x', laser_x,
            '--y', laser_y,
            '--z', laser_z,
            '--roll', laser_roll,
            '--pitch', laser_pitch,
            '--yaw', laser_yaw,
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    carto_pose_publisher_node = Node(
        package='car_localization',
        executable='carto_pose_publisher_node',
        name='carto_pose_publisher_node',
        output='screen',
        condition=IfCondition(start_pose_publisher),
        parameters=[{
            'global_frame_id': 'map',
            'base_frame_id': 'base_link',
            'output_frame_id': 'car_map',
            'pose_topic': pose_topic,
            'publish_rate_hz': pose_publish_rate_hz,
            'yaw_offset_rad': pose_yaw_offset_rad,
        }],
    )

    path_planner_node = Node(
        package='path_planner',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=[{
            'global_frame_id': 'map',
            'base_frame_id': 'base_link',
            'obstacle_file': obstacle_file,
        }],
    )

    path_controller_node = Node(
        package='path_controller',
        executable='path_controller_node',
        name='path_controller_node',
        output='screen',
        condition=IfCondition(start_navigation),
        parameters=[{
            'global_frame_id': 'map',
            'base_frame_id': 'base_link',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_stm', default_value='true'),
        DeclareLaunchArgument('start_carto', default_value='true'),
        DeclareLaunchArgument('start_navigation', default_value='true'),
        DeclareLaunchArgument('start_pose_publisher', default_value='true'),
        DeclareLaunchArgument(
            'load_state_filename',
            default_value='/home/t/car_ws/carto/map/my_map.pbstream',
        ),
        DeclareLaunchArgument('obstacle_file', default_value=''),
        DeclareLaunchArgument('pose_topic', default_value='/car/pose'),
        DeclareLaunchArgument('pose_publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('pose_yaw_offset_rad', default_value='0.0'),
        DeclareLaunchArgument('stm_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('stm_baudrate', default_value='576000'),
        DeclareLaunchArgument('laser_x', default_value='0.055'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.015'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='-1.5708'),
        stm_node,
        lidar_launch,
        base_to_laser_tf,
        carto_localization_launch,
        carto_pose_publisher_node,
        path_planner_node,
        path_controller_node,
    ])



'''
source install/setup.bash
ros2 launch car_bringup localization_control.launch.py load_state_filename:=/home/t/car_ws/carto/map/my_map.pbstream
'''
