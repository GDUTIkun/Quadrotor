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
    start_pose_publisher = LaunchConfiguration('start_pose_publisher')
    start_occupancy_grid = LaunchConfiguration('start_occupancy_grid')

    stm_port = LaunchConfiguration('stm_port')
    stm_baudrate = LaunchConfiguration('stm_baudrate')
    debug_stm_rx = LaunchConfiguration('debug_stm_rx')
    pose_topic = LaunchConfiguration('pose_topic')
    target_pose_topic = LaunchConfiguration('target_pose_topic')
    carto_odom_topic = LaunchConfiguration('carto_odom_topic')
    pose_publish_rate_hz = LaunchConfiguration('pose_publish_rate_hz')
    pose_yaw_offset_rad = LaunchConfiguration('pose_yaw_offset_rad')
    pose_filter_tau_s = LaunchConfiguration('pose_filter_tau_s')
    pose_velocity_filter_tau_s = LaunchConfiguration('pose_velocity_filter_tau_s')
    car_namespace = LaunchConfiguration('car_namespace')
    carto_map_frame = LaunchConfiguration('carto_map_frame')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    laser_frame = LaunchConfiguration('laser_frame')
    imu_frame = LaunchConfiguration('imu_frame')
    scan_topic = LaunchConfiguration('scan_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    stm_status_topic = LaunchConfiguration('stm_status_topic')
    wheel_odom_topic = LaunchConfiguration('wheel_odom_topic')

    laser_x = LaunchConfiguration('laser_x')
    laser_y = LaunchConfiguration('laser_y')
    laser_z = LaunchConfiguration('laser_z')
    laser_roll = LaunchConfiguration('laser_roll')
    laser_pitch = LaunchConfiguration('laser_pitch')
    laser_yaw = LaunchConfiguration('laser_yaw')
    imu_x = LaunchConfiguration('imu_x')
    imu_y = LaunchConfiguration('imu_y')
    imu_z = LaunchConfiguration('imu_z')
    imu_roll = LaunchConfiguration('imu_roll')
    imu_pitch = LaunchConfiguration('imu_pitch')
    imu_yaw = LaunchConfiguration('imu_yaw')
    target_frame = LaunchConfiguration('target_frame')
    target_x = LaunchConfiguration('target_x')
    target_y = LaunchConfiguration('target_y')
    target_z = LaunchConfiguration('target_z')
    target_roll = LaunchConfiguration('target_roll')
    target_pitch = LaunchConfiguration('target_pitch')
    target_yaw = LaunchConfiguration('target_yaw')

    stm_node = Node(
        package='stm_bridge',
        executable='stm_bridge_node',
        name='stm_bridge_node',
        output='screen',
        condition=IfCondition(start_stm),
        parameters=[{
            'port': stm_port,
            'baudrate': stm_baudrate,
            'debug_rx_hex': debug_stm_rx,
            'base_frame_id': base_frame,
            'odom_frame_id': odom_frame,
            'imu_frame_id': imu_frame,
            'cmd_vel_topic': cmd_vel_topic,
            'imu_topic': imu_topic,
            'status_topic': stm_status_topic,
            'wheel_odom_topic': wheel_odom_topic,
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
        launch_arguments={
            'frame_id': laser_frame,
            'scan_topic': scan_topic,
        }.items(),
    )

    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carto'),
                'launch',
                'my_laser_with_imu.launch.py',
            ])
        ),
        condition=IfCondition(start_carto),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_occupancy_grid': start_occupancy_grid,
            'car_namespace': car_namespace,
            'scan_topic': scan_topic,
            'imu_topic': imu_topic,
        }.items(),
    )

    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='car_base_link_to_laser_tf',
        output='screen',
        arguments=[
            '--x', laser_x,
            '--y', laser_y,
            '--z', laser_z,
            '--roll', laser_roll,
            '--pitch', laser_pitch,
            '--yaw', laser_yaw,
            '--frame-id', base_frame,
            '--child-frame-id', laser_frame,
        ],
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='car_base_link_to_imu_tf',
        output='screen',
        arguments=[
            '--x', imu_x,
            '--y', imu_y,
            '--z', imu_z,
            '--roll', imu_roll,
            '--pitch', imu_pitch,
            '--yaw', imu_yaw,
            '--frame-id', base_frame,
            '--child-frame-id', imu_frame,
        ],
    )

    base_to_target_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='car_base_link_to_target_tf',
        output='screen',
        arguments=[
            '--x', target_x,
            '--y', target_y,
            '--z', target_z,
            '--roll', target_roll,
            '--pitch', target_pitch,
            '--yaw', target_yaw,
            '--frame-id', base_frame,
            '--child-frame-id', target_frame,
        ],
    )

    carto_pose_publisher_node = Node(
        package='car_localization',
        executable='carto_pose_publisher_node',
        name='car_carto_pose_publisher_node',
        output='screen',
        condition=IfCondition(start_pose_publisher),
        parameters=[{
            'global_frame_id': carto_map_frame,
            'base_frame_id': base_frame,
            'output_frame_id': 'car_map',
            'pose_topic': pose_topic,
            'target_frame_id': target_frame,
            'target_pose_topic': target_pose_topic,
            'publish_target_pose': True,
            'odom_topic': carto_odom_topic,
            'odom_child_frame_id': base_frame,
            'publish_rate_hz': pose_publish_rate_hz,
            'yaw_offset_rad': pose_yaw_offset_rad,
            'publish_odom': True,
            'pose_filter_tau_s': pose_filter_tau_s,
            'velocity_filter_tau_s': pose_velocity_filter_tau_s,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_stm', default_value='true'),
        DeclareLaunchArgument('start_carto', default_value='true'),
        DeclareLaunchArgument('start_pose_publisher', default_value='true'),
        DeclareLaunchArgument('start_occupancy_grid', default_value='true'),
        DeclareLaunchArgument('pose_topic', default_value='/car/pose'),
        DeclareLaunchArgument('target_pose_topic', default_value='/car/target_pose'),
        DeclareLaunchArgument('carto_odom_topic', default_value='/car/odom/carto'),
        DeclareLaunchArgument('pose_publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('pose_yaw_offset_rad', default_value='0.0'),
        DeclareLaunchArgument('pose_filter_tau_s', default_value='0.10'),
        DeclareLaunchArgument('pose_velocity_filter_tau_s', default_value='0.25'),
        DeclareLaunchArgument('car_namespace', default_value='car'),
        DeclareLaunchArgument('carto_map_frame', default_value='car_carto_map'),
        DeclareLaunchArgument('odom_frame', default_value='car_odom'),
        DeclareLaunchArgument('base_frame', default_value='car_base_link'),
        DeclareLaunchArgument('laser_frame', default_value='car_laser'),
        DeclareLaunchArgument('imu_frame', default_value='car_imu_link'),
        DeclareLaunchArgument('scan_topic', default_value='/car/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/car/imu/data_valid'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('stm_status_topic', default_value='/car/stm/status'),
        DeclareLaunchArgument('wheel_odom_topic', default_value='/car/odom/wheel'),
        DeclareLaunchArgument('stm_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('stm_baudrate', default_value='576000'),
        DeclareLaunchArgument('debug_stm_rx', default_value='false'),
        DeclareLaunchArgument('laser_x', default_value='0.06842'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.1246'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='-1.5708'),
        DeclareLaunchArgument('imu_x', default_value='0.0'),
        DeclareLaunchArgument('imu_y', default_value='-0.0146'),
        DeclareLaunchArgument('imu_z', default_value='0.075'),
        DeclareLaunchArgument('imu_roll', default_value='0.0'),
        DeclareLaunchArgument('imu_pitch', default_value='0.0'),
        DeclareLaunchArgument('imu_yaw', default_value='0.0'),
        DeclareLaunchArgument('target_frame', default_value='target'),
        DeclareLaunchArgument('target_x', default_value='0.09553'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='0.0'),
        DeclareLaunchArgument('target_roll', default_value='0.0'),
        DeclareLaunchArgument('target_pitch', default_value='0.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        stm_node,
        lidar_launch,
        base_to_laser_tf,
        base_to_imu_tf,
        base_to_target_tf,
        carto_launch,
        carto_pose_publisher_node,
    ])
