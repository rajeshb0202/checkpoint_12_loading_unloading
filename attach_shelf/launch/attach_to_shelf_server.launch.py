import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('attach_shelf')

    approach_server_action = Node(
        package="attach_shelf",
        executable="approach_service_server_node",
        output="screen",
        name= "approach_service_server_node",
    )

    return LaunchDescription([
        approach_server_action,        
    ])
