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

    carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('carto'),
                'launch',
                'my_laser_with_imu.launch.py',
            ])
        ),
        condition=IfCondition(start_carto),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
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

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        DeclareLaunchArgument('start_stm', default_value='true'),
        DeclareLaunchArgument('start_carto', default_value='true'),
        DeclareLaunchArgument('stm_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument('stm_baudrate', default_value='576000'),
        DeclareLaunchArgument('laser_x', default_value='0.055'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.015'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        stm_node,
        lidar_launch,
        base_to_laser_tf,
        carto_launch,
    ])
