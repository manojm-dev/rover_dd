import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the package share directory
    pkg_share = FindPackageShare('questbot_gazebo').find('questbot_gazebo')

    # File paths
    default_world_path = os.path.join(pkg_share, 'worlds', 'warehouse.world')

    # Launch configurations
    world = LaunchConfiguration('world')
    gazebo_version = LaunchConfiguration('gazebo_version')

    # Declare launch arguments
    declare_launch_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use gazebo clock'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path,
            description='Absolute path of Gazebo world file'
        ),
        DeclareLaunchArgument(
            name='gazebo_version',
            default_value='gazebo',
            choices=['gazebo', 'gzsim'],
            description='Choose which version of gazebo to use'
        ),
    ]

    # Include classic Gazebo simulation launch file
    start_classic_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo_simulation.launch.py')),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(PythonExpression(["'", gazebo_version, "' == 'gazebo'"]))
    )

    # Include Gazebo Sim (gz-sim) launch file
    start_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gzsim_simulation.launch.py')),
        launch_arguments={'world': world}.items(),
        condition=IfCondition(PythonExpression(["'", gazebo_version, "' == 'gzsim'"]))
    )

    # Return the complete launch description
    return LaunchDescription(
        declare_launch_arguments + [
            start_classic_gazebo,
            start_gz_sim
        ]
    )
