from setuptools import setup
from glob import glob
import os

package_name = 'robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Artem',
    maintainer_email='artem@example.com',
    description='Robot bringup with lidar obstacle avoidance',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'lidar_obstacle_node = robot_bringup.lidar_obstacle_node:main',
            'ros_stream = robot_bringup.ros_stream:main',
        ],
    },
)
