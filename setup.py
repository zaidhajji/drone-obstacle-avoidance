import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drone_avoidance'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zaid',
    maintainer_email='you@example.com',
    description='A* + lidar obstacle avoidance for a drone in ROS 2 and Gazebo',
    license='MIT',
    entry_points={
        'console_scripts': [
            'avoid_node = drone_avoidance.avoid_node:main',
            'nav_node = drone_avoidance.nav_node:main',
            'generate_world = drone_avoidance.generate_world:main',
        ],
    },
)
