import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Packages share directory
    pkg_share = FindPackageShare('questbot_navigation').find('questbot_navigation')
    pkg_localization = FindPackageShare('questbot_localization').find('questbot_localization')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup').find('nav2_bringup')

    # Files paths 
    default_param_file = os.path.join(pkg_share, 'config/nav2_params.yaml')
    default_rviz_config_file = os.path.join(pkg_share, 'rviz/navigation.rviz')
    default_map = os.path.join(pkg_localization, 'maps/factory.yaml')
    

    # Create Launch configuration variables 
    use_sim_time = LaunchConfiguration('use_sim_time')
        
    # Launch configuration variables specific to nav2 bringup
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map_yaml_file')
    params_file = LaunchConfiguration('params_file')
    namespace = LaunchConfiguration('namespace')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    
    
    # Launch configuration variables specific to simulation
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [

        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        
        DeclareLaunchArgument(
            name='slam', 
            default_value='False', 
            description='Whether run a SLAM'
        ),

        DeclareLaunchArgument(  
            name='params_file',
            default_value=default_param_file,
            description='Navigation2 parameter file'
        ),
        
        DeclareLaunchArgument(  
            name='map_yaml_file',
            default_value=default_map,
            description='Map file'
        ),

        DeclareLaunchArgument(
            name='namespace', 
            default_value='', 
            description='Top-level namespace'
        ),
        
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack',
        ),

        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Whether to use composed bringup',
        ),
        
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes. Applied when composition is disabled.',
        ),
        
        DeclareLaunchArgument(  
            name='use_rviz',
            default_value='true',
            description='Use Rviz Visualization tools'
        ),
        
        DeclareLaunchArgument(  
            name='rviz_config_file',
            default_value=default_rviz_config_file,
            description='Use Rviz Visualization tools'
        )
    ]

    # Robot state publisher
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            }.items()
    )

    # Start RViz
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch/rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config_file,
        }.items(),
    )


    return LaunchDescription(
        declare_arguments + [
            rviz_cmd,
            start_navigation,
        ] 
    )