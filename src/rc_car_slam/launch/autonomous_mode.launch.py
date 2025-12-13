import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lidar_pkg = get_package_share_directory('sllidar_ros2')
    desc_pkg = get_package_share_directory('rc_car_description')
    slam_pkg = get_package_share_directory('rc_car_slam')
    bridge_pkg = get_package_share_directory('foxglove_bridge')
    
    urdf_file = os.path.join(desc_pkg, 'urdf', 'rc_car.urdf')
    slam_config = os.path.join(slam_pkg, 'config')

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

        # 3. IMU
        Node(
            package='rc_car_sensors',
            executable='imu_node',
            name='imu_node',
            output='screen',
        ),

        # 4. Cartographer localization with landmarks
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_node',
                    name='cartographer_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}],
                    arguments=[
                        '-configuration_directory', slam_config,
                        '-configuration_basename', 'cartographer_localization.lua',
                        '-load_state_filename', '/home/bench207/ros2_ws/maps/my_map.pbstream',
                    ],
                    remappings=[
                        ('scan', '/scan'),
                        ('imu', '/imu'),
                        ('landmarks', '/landmarks'),
                    ]
                ),
            ]
        ),

        # 5. Occupancy grid
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='cartographer_occupancy_grid_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'resolution': 0.05,
                    }],
                ),
            ]
        ),

        # 6. Camera with landmarks enabled
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rc_car_sensors',
                    executable='camera_apriltag_node',
                    name='camera_apriltag_node',
                    output='screen',
                    parameters=[{
                        'width': 640,
                        'height': 480,
                        'fps': 15,
                        'sounds_directory': '/home/bench207/ros2_ws/sounds',
                        'tag_size': 0.1,
                        'publish_tag_markers': False,
                        'save_tags': False,
                        'publish_landmarks': True,
                        'known_tags_file': '/home/bench207/ros2_ws/tag_positions.json',
                        'play_sounds': False,
                    }]
                ),
            ]
        ),

        # 7. Motor Controller
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rc_car_sensors',
                    executable='motor_controller_node',
                    name='motor_controller_node',
                    output='screen',
                    parameters=[{
                        'motor_pin': 13,
                        'servo_pin': 12,
                        'max_speed': 0.5,
                        'max_steering_angle': 0.5,
                        'motor_min_duty': 85.0,     # 50% = max speed
                        'motor_max_duty': 100.0,    # 100% = stopped
                        'servo_center_duty': 7.5,   # 7.5% = center
                        'servo_range_duty': 2.5,    # +/- 2.5%
                        'timeout': 0.5,
                    }]
                ),
            ]
        ),

        # 8. Autonomous Navigation (MOTORS ENABLED)
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='rc_car_sensors',
                    executable='autonomous_nav_node',
                    name='autonomous_nav_node',
                    output='screen',
                    parameters=[{
                        'tags_file': '/home/bench207/ros2_ws/tag_positions.json',
                        'map_yaml': '/home/bench207/ros2_ws/maps/my_map.yaml',
                        'goal_tolerance': 0.5,
                        'waypoint_tolerance': 0.4,
                        'max_linear_speed': 0.5,
                        'max_angular_speed': 0.5,
                        'min_linear_speed': 0.25,       
                        'turn_slowdown_factor': 0.3,    
                        'obstacle_inflation': 0.25,      # Increased slightly for more clearance
                        'start_delay': 15.0,
                        'enable_motor_control': True,
                        'sounds_directory': '/home/bench207/ros2_ws/sounds',
                    }]
                ),
            ]
        ),

        # 9. Foxglove Bridge
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(
                os.path.join(bridge_pkg, 'launch', 'foxglove_bridge_launch.xml')
            )
        ),
    ])