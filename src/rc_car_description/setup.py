from setuptools import setup
import os
from glob import glob

package_name = 'rc_car_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install URDF/xacro files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bench207',
    maintainer_email='',
    description='Description package for RC car robot (URDF + TF)',
    license='',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
