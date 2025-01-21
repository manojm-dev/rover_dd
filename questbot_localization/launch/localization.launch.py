import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_share = FindPackageShare('questbot_localization').find('questbot_localization')

    # Files paths
    rviz_config = os.path.join(pkg_share, 'rviz/slam.rviz')

    # Launch configuration 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    sync =LaunchConfiguration('sync')
    localization = LaunchConfiguration('localization')


    # Launch Arguments 
    declare_arguments = [

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        
        DeclareLaunchArgument(
            name='sync',
            default_value='true',
            choices=['true', 'false'],
            description='Synchronous mode'
        ),
        
        DeclareLaunchArgument(
            name='localization',
            default_value='false',
            choices=['true', 'false'],
            description='Localization mode'
        ),
        
        DeclareLaunchArgument(
            name='use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Use Rviz Visualization tool'
        ),
    ]

    start_slamtb_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam_online_async.launch.py')), 
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization
            }.items(),
        condition=UnlessCondition(sync)
    )
    
    start_slamtb_sync = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'slam_online_sync.launch.py')), 
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization
            }.items(),
        condition=IfCondition(sync)
    )


    # Start RViz
    start_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': True
        }],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription(
        declare_arguments + [
            start_slamtb_async,
            start_slamtb_sync,
            start_rviz
        ]
    )
