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

    # Files paths
    default_world_path = os.path.join(pkg_share, 'worlds/empty.world')
    models_path = os.path.join(pkg_share, 'models')

    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    verbose = LaunchConfiguration('verbose')
    verbose_level = LaunchConfiguration('verbose_level')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))

    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use gazebo clock'
        ),
        DeclareLaunchArgument(
            name='world',
            default_value=default_world_path,
            description='Absolute path of gazebo WORLD file'
        ),
        DeclareLaunchArgument(
            name='verbose',
            default_value='false',
            choices=['true', 'false'],
            description='Turn on/off verbose'
        ),
        DeclareLaunchArgument(
            name='verbose_level',
            default_value='debug',
            choices=['debug', 'info', 'warn', 'error'],
            description='Mention verbose level'
        ),
    ]

    # Open gazebo environment
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'verbose': verbose,
            'level': verbose_level
        }.items(),
    )

    return LaunchDescription(
        declare_arguments + [
            start_gazebo
        ]
    )