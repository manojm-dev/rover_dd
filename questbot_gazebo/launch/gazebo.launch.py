import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_gazebo').find('questbot_gazebo')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros').find('gazebo_ros')
    
    default_world_file = os.path.join(pkg_share, 'worlds', 'cones.world')
    models_path = os.path.join(pkg_share, 'models')
    
    world = LaunchConfiguration('world')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path
    
    declare_launch_args = [
        
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_file,
            description='Gazebo world file'
        )
        
    ]

    # Open gazebo environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'))
    )

    return LaunchDescription(
        declare_launch_args + [
            start_gazebo
        ]
    )