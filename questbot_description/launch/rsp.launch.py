import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Packages share directory
    pkg_share = FindPackageShare('questbot_description').find('questbot_description')

    # File paths
    robot_2w_diffdrive = os.path.join(pkg_share, 'urdf/2w_diffdrive/robot.urdf.xacro')
    robot_6w_diffdrive = os.path.join(pkg_share, 'urdf/6w_diffdrive/robot.urdf.xacro')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    drive_type = LaunchConfiguration('drive_type')
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_gzsim = LaunchConfiguration('use_gzsim')

    # Default launch configuration arguments
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
        DeclareLaunchArgument(
            name='drive_type',
            default_value='2w_diffdrive',
            choices=['2w_diffdrive', '6w_diffdrive'],
            description='Choose between 2WD and 6WD robot configuration'
        ),
        DeclareLaunchArgument(
            name='use_gazebo',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo classic'
        ),
        DeclareLaunchArgument(
            name='use_gzsim',
            default_value='false',
            choices=['true', 'false'],
            description='Use Gazebo Sim'
        ),
    ]

    # Nodes for robot configurations
    robot_state_publisher_2w = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_2w',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', robot_2w_diffdrive,
                ' use_gazebo:=', use_gazebo,
                ' use_gzsim:=', use_gzsim
            ])
        }],
        condition=IfCondition(PythonExpression(["'", drive_type, "' == '2w_diffdrive'"]))
    )

    robot_state_publisher_6w = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_6w',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                'xacro ', robot_6w_diffdrive,
                ' use_gazebo:=', use_gazebo,
                ' use_gzsim:=', use_gzsim
            ])
        }],
        condition=IfCondition(PythonExpression(["'", drive_type, "' == '6w_diffdrive'"]))
    )

    return LaunchDescription(declare_arguments + [
        GroupAction([
            robot_state_publisher_2w,
            robot_state_publisher_6w,
        ]),
    ])
