import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    slam_pkg = get_package_share_directory('rc_car_slam')
    sensors_pkg = get_package_share_directory('rc_car_sensors')

    return LaunchDescription([
        # Launch SLAM (LiDAR + IMU + Cartographer + Foxglove)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_pkg, 'launch', 'cartographer.launch.py')
            )
        ),

        # Launch camera + AprilTag detection (with delay to let SLAM start)
        TimerAction(
            period=5.0,
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
                        'tag_size': 0.1,  # Adjust to your printed tag size in meters
                    }]
                ),
            ]
        ),
    ])