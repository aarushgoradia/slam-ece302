import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Locate the files
    slam_pkg_path = get_package_share_directory('slam_toolbox')
    desc_pkg_path = get_package_share_directory('rc_car_description')
    
    urdf_file = os.path.join(desc_pkg_path, 'urdf', 'rc_car.urdf.xacro')
    slam_config = os.path.join(get_package_share_directory('rc_car_slam'), 'config', 'slam.yaml')

    return LaunchDescription([
        # 2. Robot State Publisher (Publishes the physical description)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_file).read()}]
        ),

        # 3. SLAM Toolbox (The Mapper)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        )
    ])