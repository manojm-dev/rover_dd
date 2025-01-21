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
    # rviz_config = os.path.join(pkg_share, 'rviz/display.rviz')

    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose')
    verbose_level = LaunchConfiguration('verbose_level')
    use_rviz = LaunchConfiguration('use_rviz')  

    
    declare_arguments = [
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path, 
            description='Absolute path of gazebo WORLD file'
        ),

        DeclareLaunchArgument(  
            name='verbose',
            default_value='true',
            description='Set "true" to increase messages written to terminal.'
        ),
        
        DeclareLaunchArgument(
            name='verbose_level',
            default_value='debug',
            choices=['debug', 'info', 'warn', 'error'],
            description='Mention verbose level'
        ),

        DeclareLaunchArgument(  
            name='use_rviz',
            default_value='false',
            description='To open rviz tool'
        ),
    ]


    # Open simulation environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world'         : world,
            'verbose'       : verbose,
            'verbose_level' : verbose_level
        }.items()
    )

    # Spawn robot in simulation environment
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo_spawn.launch.py')),
    )

    # # Start RViz
    # start_rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config],
    #     parameters=[{
    #         'use_sim_time': True
    #     }],
    #     condition=IfCondition(use_rviz)
    # )


    return LaunchDescription(
        declare_arguments + [
            start_gazebo,
            spawn_robot,
            # start_rviz
        ] 
    )