import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_share = FindPackageShare('questbot_localization').find('questbot_localization')
    pkg_slamtb_share = FindPackageShare('slam_toolbox').find('slam_toolbox')

    # File paths
    slam_config_mapping = os.path.join(pkg_share, 'config/mapper_params_online_async_mapping.yaml')
    slam_config_locazlization = os.path.join(pkg_share, 'config/mapper_params_online_async_localization.yaml')

    # Launch configuration variables with default values
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        
        DeclareLaunchArgument(
            name='localization',
            default_value='false',
            choices=['true', 'false'],
            description='Switch between localization and mapping mode'
        ),
    ]

    start_slamtb_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slamtb_share, 'launch', 'online_async_launch.py')), 
        launch_arguments={
            'slam_params_file': slam_config_mapping,
            'use_sim_time': use_sim_time
            }.items(),
        condition=UnlessCondition(localization)
    )
    
    start_slamtb_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_slamtb_share, 'launch', 'online_async_launch.py')), 
        launch_arguments={
            'slam_params_file': slam_config_locazlization,
            'use_sim_time': use_sim_time
            }.items(),
        condition=IfCondition(localization)
    )

    return LaunchDescription(
        declare_arguments + [
            start_slamtb_mapping,
            start_slamtb_localization
        ]
    )