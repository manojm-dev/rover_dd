import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_gazebo').find('questbot_gazebo')

    # Files paths
    default_world_path = os.path.join(pkg_share, 'worlds/warehouse.world')
    models_path = os.path.join(pkg_share, 'models')

    world = LaunchConfiguration('world')

    declare_arguments = [
        
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path,
            description='Absolute path of gazebo WORLD file'
        )

    ]
    
    # Open simulation environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Spawn robot in simulation environment
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo_spawn.launch.py')),
    )

    return LaunchDescription(
        declare_arguments + [
            start_gazebo,
            spawn_robot,
        ]
    )
