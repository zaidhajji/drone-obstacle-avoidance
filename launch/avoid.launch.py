import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('drone_avoidance')
    gz = get_package_share_directory('gazebo_ros')
    world = os.path.join(pkg, 'worlds', 'obstacles.world')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world}.items()),
        Node(package='drone_avoidance', executable='avoid_node', output='screen'),
    ])
