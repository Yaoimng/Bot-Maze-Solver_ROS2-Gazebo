import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('wall_follower_sim')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'maze.world')

    return LaunchDescription([
        # 1. Jalankan Gazebo dengan World kita
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 2. Spawn Robot ke Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'mazebot', '-file', urdf_file, '-x', '0', '-y', '0', '-z', '0.1'],
            output='screen'
        ),

        # 3. Jalankan Node Wall Follower Logic
        Node(
            package='wall_follower_sim',
            executable='wall_follower',
            name='wall_follower',
            output='screen'
        )
    ])