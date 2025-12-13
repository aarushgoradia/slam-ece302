import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lidar_pkg = get_package_share_directory('sllidar_ros2')
    desc_pkg = get_package_share_directory('rc_car_description')
    slam_pkg = get_package_share_directory('rc_car_slam')
    bridge_pkg = get_package_share_directory('foxglove_bridge')
    
    urdf_file = os.path.join(desc_pkg, 'urdf', 'rc_car.urdf')
    cartographer_config = os.path.join(slam_pkg, 'config')
    
    return LaunchDescription([
        # 1. LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'sllidar_a1_launch.py')
            )
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 3. IMU Node
        Node(
            package='rc_car_sensors',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),

        # 4. Cartographer
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_node',
                    name='cartographer_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                    arguments=[
                        '-configuration_directory', cartographer_config,
                        '-configuration_basename', 'cartographer.lua'
                    ],
                    remappings=[
                        ('scan', '/scan'),
                        ('imu', '/imu'),
                    ]
                ),
            ]
        ),

        # 5. Occupancy Grid (publishes /map)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                    arguments=['-resolution', '0.05']
                ),
            ]
        ),

        # 6. Foxglove Bridge
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(bridge_pkg, 'launch', 'foxglove_bridge_launch.xml')
            )
        ),
    ])