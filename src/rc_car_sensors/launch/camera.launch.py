from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Combined camera + AprilTag detector
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
            }]
        ),
    ])