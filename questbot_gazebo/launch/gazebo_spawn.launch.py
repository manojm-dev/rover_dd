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
    description_share = FindPackageShare('questbot_description').find('questbot_description')
    gazebo_share = FindPackageShare('gazebo_ros').find('gazebo_ros')

    # Launch configuration variables with default values 
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Launch configuration with file paths
    world = LaunchConfiguration('world')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
    ]

    # Robot state publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(description_share, 'launch', 'rsp.launch.py')),
        launch_arguments={
            'use_sim_time'    : use_sim_time,
            'use_gazebo'      : 'true',
            'use_gzsim'       : 'false',
            'drive_type'      : '6w_diffdrive'
        }.items()
    )

    spawn_entity_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        name="spawn_entity",
        output='screen',
        arguments=['-topic', 'robot_description', '-entity',  'questbot_2wd', '-z', '0.5', '-x', '0.48028860739951745', '-y', '-7.475515855306923'],
        parameters=[{
            'use_sim_time': use_sim_time
            }]
    )


    return LaunchDescription(
        declare_arguments + [
            robot_state_publisher,
            spawn_entity_node
        ] 
    )