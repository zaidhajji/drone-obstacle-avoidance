import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def maybe_rviz(context):
    if LaunchConfiguration('rviz').perform(context).lower() != 'true':
        return []
    pkg = get_package_share_directory('drone_avoidance')
    cfg = os.path.join(pkg, 'rviz', 'drone.rviz')
    args = ['-d', cfg] if os.path.exists(cfg) else []
    return [Node(package='rviz2', executable='rviz2', arguments=args)]


def generate_launch_description():
    pkg = get_package_share_directory('drone_avoidance')
    gz = get_package_share_directory('gazebo_ros')
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='complex.world'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gazebo.launch.py')),
            launch_arguments={'world': PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')]),
                              'gui': LaunchConfiguration('gui')}.items()),
        Node(package='drone_avoidance', executable='nav_node', output='screen',
             parameters=[os.path.join(pkg, 'config', 'params.yaml'),
                         {'map_file': os.path.join(pkg, 'config', 'map.yaml')}]),
        OpaqueFunction(function=maybe_rviz),
    ])
