import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_butlerbot_localization = FindPackageShare('butlerbot_localization').find('butlerbot_localization')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # File paths
    default_param_file = os.path.join(pkg_butlerbot_localization, 'config/localization.yaml')
    default_map_file = os.path.join(pkg_butlerbot_localization, 'maps/factory.yaml')

    # Launch configuration variables with default values
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')

    # Launch configuration with file paths
    param_file = LaunchConfiguration('param_file')
    map_file = LaunchConfiguration('map_file')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='namespace',
            default_value='butlerbot',
            description='Robot Namespace'
        ),
        DeclareLaunchArgument(
            name='param_file',
            default_value=default_param_file,
            description='Absolute path of amcl config file'
        ),
        DeclareLaunchArgument(
            name='map_file',
            default_value=default_map_file,
            description='Absolute path of map file'
        ),
    ]

    start_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map'           : map_file,
            'use_sim_time'  : use_sim_time,
        }.items()
    )

    return LaunchDescription(
        declare_arguments + [
            start_amcl
        ]
    )