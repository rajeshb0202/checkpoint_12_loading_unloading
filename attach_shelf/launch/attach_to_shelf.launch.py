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

    obstacle_arg = DeclareLaunchArgument("obstacle", default_value="0.3")
    degrees_arg = DeclareLaunchArgument("degrees", default_value="-90")
    final_approach_arg = DeclareLaunchArgument("final_approach", default_value="true")

    obstacle_ = LaunchConfiguration("obstacle")
    degrees_ = LaunchConfiguration("degrees")
    final_approach_ = LaunchConfiguration("final_approach")

    pre_approach_v2_action = Node(
        package="attach_shelf",
        executable="pre_approach_v2_node",
        output="screen",
        name= "pre_approach_v2_node",
        parameters=[{"obstacle": obstacle_,
                    "degrees": degrees_ ,
                    "final_approach": final_approach_}]
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

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        rviz_action,
        approach_server_action,
        pre_approach_v2_action,
        
    ])
