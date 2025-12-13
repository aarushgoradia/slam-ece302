import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rc_car_sensors'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bench207',
    maintainer_email='bench207@todo.todo',
    description='RC car sensor nodes',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = rc_car_sensors.imu_node:main',
            'camera_apriltag_node = rc_car_sensors.camera_apriltag_node:main',
            'motor_controller_node = rc_car_sensors.motor_controller_node:main',
            'autonomous_nav_node = rc_car_sensors.autonomous_nav_node:main',
            'map_saver_node = rc_car_sensors.map_saver_node:main',
            'path_test_node = rc_car_sensors.path_test_node:main',
        ],
    },
)