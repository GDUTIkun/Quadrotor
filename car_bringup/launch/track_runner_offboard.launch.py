from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    start_mapping = LaunchConfiguration('start_mapping')
    start_track_runner = LaunchConfiguration('start_track_runner')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_lidar = LaunchConfiguration('start_lidar')
    start_stm = LaunchConfiguration('start_stm')
    start_carto = LaunchConfiguration('start_carto')
    start_pose_publisher = LaunchConfiguration('start_pose_publisher')
    start_occupancy_grid = LaunchConfiguration('start_occupancy_grid')

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
    stm_port = LaunchConfiguration('stm_port')
    stm_baudrate = LaunchConfiguration('stm_baudrate')
    debug_stm_rx = LaunchConfiguration('debug_stm_rx')

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

    straight_length_m = LaunchConfiguration('straight_length_m')
    radius_m = LaunchConfiguration('radius_m')
    path_spacing_m = LaunchConfiguration('path_spacing_m')
    default_speed_m_s = LaunchConfiguration('default_speed_m_s')
    default_laps = LaunchConfiguration('default_laps')
    lookahead_distance_m = LaunchConfiguration('lookahead_distance_m')
    waypoint_tolerance_m = LaunchConfiguration('waypoint_tolerance_m')
    goal_tolerance_m = LaunchConfiguration('goal_tolerance_m')
    k_w = LaunchConfiguration('k_w')
    k_w_ff_speed_intercept = LaunchConfiguration('k_w_ff_speed_intercept')
    k_w_ff_speed_slope = LaunchConfiguration('k_w_ff_speed_slope')
    k_w_ff_min = LaunchConfiguration('k_w_ff_min')
    k_w_ff_max = LaunchConfiguration('k_w_ff_max')
    k_w_rate = LaunchConfiguration('k_w_rate')
    k_i_rate = LaunchConfiguration('k_i_rate')
    k_d_rate = LaunchConfiguration('k_d_rate')
    w_error_integral_max = LaunchConfiguration('w_error_integral_max')
    w_error_derivative_filter_tau_s = LaunchConfiguration('w_error_derivative_filter_tau_s')
    w_max_rad_s = LaunchConfiguration('w_max_rad_s')
    pose_timeout_s = LaunchConfiguration('pose_timeout_s')
    angular_velocity_timeout_s = LaunchConfiguration('angular_velocity_timeout_s')
    use_angular_velocity_feedback = LaunchConfiguration('use_angular_velocity_feedback')
    use_start_pose_as_origin = LaunchConfiguration('use_start_pose_as_origin')
    control_rate_hz = LaunchConfiguration('control_rate_hz')
    command_topic = LaunchConfiguration('command_topic')
    speed_topic = LaunchConfiguration('speed_topic')
    laps_topic = LaunchConfiguration('laps_topic')
    status_topic = LaunchConfiguration('status_topic')

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('car_bringup'),
                'launch',
                'mapping.launch.py',
            ])
        ),
        condition=IfCondition(start_mapping),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_lidar': start_lidar,
            'start_stm': start_stm,
            'start_carto': start_carto,
            'start_pose_publisher': start_pose_publisher,
            'start_occupancy_grid': start_occupancy_grid,
            'pose_topic': pose_topic,
            'target_pose_topic': target_pose_topic,
            'carto_odom_topic': carto_odom_topic,
            'pose_publish_rate_hz': pose_publish_rate_hz,
            'pose_yaw_offset_rad': pose_yaw_offset_rad,
            'pose_filter_tau_s': pose_filter_tau_s,
            'pose_velocity_filter_tau_s': pose_velocity_filter_tau_s,
            'car_namespace': car_namespace,
            'carto_map_frame': carto_map_frame,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'laser_frame': laser_frame,
            'imu_frame': imu_frame,
            'scan_topic': scan_topic,
            'imu_topic': imu_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'stm_status_topic': stm_status_topic,
            'wheel_odom_topic': wheel_odom_topic,
            'stm_port': stm_port,
            'stm_baudrate': stm_baudrate,
            'debug_stm_rx': debug_stm_rx,
            'laser_x': laser_x,
            'laser_y': laser_y,
            'laser_z': laser_z,
            'laser_roll': laser_roll,
            'laser_pitch': laser_pitch,
            'laser_yaw': laser_yaw,
            'imu_x': imu_x,
            'imu_y': imu_y,
            'imu_z': imu_z,
            'imu_roll': imu_roll,
            'imu_pitch': imu_pitch,
            'imu_yaw': imu_yaw,
            'target_frame': target_frame,
            'target_x': target_x,
            'target_y': target_y,
            'target_z': target_z,
            'target_roll': target_roll,
            'target_pitch': target_pitch,
            'target_yaw': target_yaw,
        }.items(),
    )

    track_runner_node = Node(
        package='track_runner',
        executable='track_runner_node',
        name='track_runner_node',
        output='screen',
        condition=IfCondition(start_track_runner),
        parameters=[{
            'straight_length_m': straight_length_m,
            'radius_m': radius_m,
            'path_spacing_m': path_spacing_m,
            'default_speed_m_s': default_speed_m_s,
            'default_laps': default_laps,
            'lookahead_distance_m': lookahead_distance_m,
            'waypoint_tolerance_m': waypoint_tolerance_m,
            'goal_tolerance_m': goal_tolerance_m,
            'k_w': k_w,
            'k_w_ff_speed_intercept': k_w_ff_speed_intercept,
            'k_w_ff_speed_slope': k_w_ff_speed_slope,
            'k_w_ff_min': k_w_ff_min,
            'k_w_ff_max': k_w_ff_max,
            'k_w_rate': k_w_rate,
            'k_i_rate': k_i_rate,
            'k_d_rate': k_d_rate,
            'w_error_integral_max': w_error_integral_max,
            'w_error_derivative_filter_tau_s': w_error_derivative_filter_tau_s,
            'w_max_rad_s': w_max_rad_s,
            'pose_timeout_s': pose_timeout_s,
            'angular_velocity_timeout_s': angular_velocity_timeout_s,
            'use_angular_velocity_feedback': use_angular_velocity_feedback,
            'use_start_pose_as_origin': use_start_pose_as_origin,
            'control_rate_hz': control_rate_hz,
            'pose_topic': pose_topic,
            'odom_topic': carto_odom_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'command_topic': command_topic,
            'speed_topic': speed_topic,
            'laps_topic': laps_topic,
            'status_topic': status_topic,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('start_mapping', default_value='true'),
        DeclareLaunchArgument('start_track_runner', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_stm', default_value='true'),
        DeclareLaunchArgument('start_carto', default_value='true'),
        DeclareLaunchArgument('start_pose_publisher', default_value='true'),
        DeclareLaunchArgument('start_occupancy_grid', default_value='false'),
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
        DeclareLaunchArgument('target_x', default_value='0.18003'),
        DeclareLaunchArgument('target_y', default_value='0.0'),
        DeclareLaunchArgument('target_z', default_value='0.0'),
        DeclareLaunchArgument('target_roll', default_value='0.0'),
        DeclareLaunchArgument('target_pitch', default_value='0.0'),
        DeclareLaunchArgument('target_yaw', default_value='0.0'),
        DeclareLaunchArgument('straight_length_m', default_value='1.42'),
        DeclareLaunchArgument('radius_m', default_value='0.75'),
        DeclareLaunchArgument('path_spacing_m', default_value='0.02'),
        DeclareLaunchArgument('default_speed_m_s', default_value='0.01'),
        DeclareLaunchArgument('default_laps', default_value='1'),
        DeclareLaunchArgument('lookahead_distance_m', default_value='0.20'),
        DeclareLaunchArgument('waypoint_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('goal_tolerance_m', default_value='0.05'),
        DeclareLaunchArgument('k_w', default_value='0.54'),
        DeclareLaunchArgument('k_w_ff_speed_intercept', default_value='5.8'),
        DeclareLaunchArgument('k_w_ff_speed_slope', default_value='-28.0'),
        DeclareLaunchArgument('k_w_ff_min', default_value='0.0'),
        DeclareLaunchArgument('k_w_ff_max', default_value='10.0'),
        DeclareLaunchArgument('k_w_rate', default_value='0.30'),
        DeclareLaunchArgument('k_i_rate', default_value='0.22'),
        DeclareLaunchArgument('k_d_rate', default_value='0.0'),
        DeclareLaunchArgument('w_error_integral_max', default_value='0.5'),
        DeclareLaunchArgument('w_error_derivative_filter_tau_s', default_value='0.05'),
        DeclareLaunchArgument('w_max_rad_s', default_value='0.8'),
        DeclareLaunchArgument('pose_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('angular_velocity_timeout_s', default_value='0.35'),
        DeclareLaunchArgument('use_angular_velocity_feedback', default_value='true'),
        DeclareLaunchArgument('use_start_pose_as_origin', default_value='true'),
        DeclareLaunchArgument('control_rate_hz', default_value='50.0'),
        DeclareLaunchArgument('command_topic', default_value='/car/track_runner/command'),
        DeclareLaunchArgument('speed_topic', default_value='/car/track_runner/speed'),
        DeclareLaunchArgument('laps_topic', default_value='/car/track_runner/laps'),
        DeclareLaunchArgument('status_topic', default_value='/car/track_runner/status'),
        mapping_launch,
        track_runner_node,
    ])
