
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument

from launch_ros.substitutions import FindPackage

def generate_launch_description():
    
    pkg_share = FindPackage('questbot_bringup').find('questbot_bringup')
    
    declare_launch_arguments = [
        
        DeclareLaunchArgument(
            name='',
            default_value='',
            choices=['', ''],
            description=""
        ),
        
    ]
    
    
    retrun LaunchDescription(
        declare_launch_arguments + [
            
        ]
    )