import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('attach_shelf')

    approach_server_node = Node(
        package="attach_shelf",
        executable="approach_service_server_node",
        output="screen",
        name= "approach_service_server_node",
    )

    rviz_config_dir = os.path.join(pkg_path, 'rviz', 'cp9.rviz')

    rviz_action = Node(
        package="rviz2",
        executable="rviz2",
        output='screen',
        name="rviz_node",
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )


    return LaunchDescription(
        [
            rviz_action,
            approach_server_node,
        ]
    )