import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    slam_pkg = get_package_share_directory('slam_toolbox')
    lidar_pkg = get_package_share_directory('sllidar_ros2')
    desc_pkg = get_package_share_directory('rc_car_description')
    bridge_pkg = get_package_share_directory('foxglove_bridge')
    
    urdf_file = os.path.join(desc_pkg, 'urdf', 'rc_car.urdf')
    slam_config = os.path.join(
        get_package_share_directory('rc_car_slam'), 'config', 'slam.yaml'
    )

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

        # 3. RF2O Laser Odometry (NEW)
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='rf2o_laser_odometry',
                    executable='rf2o_laser_odometry_node',
                    name='rf2o_laser_odometry',
                    output='screen',
                    parameters=[{
                        'laser_scan_topic': '/scan',
                        'odom_topic': '/odom',
                        'publish_tf': True,
                        'base_frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'laser_frame_id': 'laser',
                        'freq': 20.0,
                        "init_pose_from_topic": ""
                    }],
                    remappings=[
                        ('laser_scan', '/scan'),  # Some versions use this topic name
                    ]
                )
            ]
        ),

        # 4. SLAM Toolbox
        TimerAction(
            period=4.0,  # Give rf2o time to start first
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_pkg, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'slam_params_file': slam_config,
                        'use_sim_time': 'false',
                    }.items(),
                )
            ]
        ),

        # 5. Foxglove Bridge
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(bridge_pkg, 'launch', 'foxglove_bridge_launch.xml')
            )
        )
    ])