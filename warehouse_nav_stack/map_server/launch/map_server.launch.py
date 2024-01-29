import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    # Declare the launch argument
    map_file_name_action = DeclareLaunchArgument(
        'map_file', 
        default_value='warehouse_map_sim.yaml'
    )
    map_file_name_config = LaunchConfiguration('map_file')
    
    # Get the package path
    map_file_pkg_path = get_package_share_directory('map_server')

    # Command to construct the full path
    map_file_path = Command([
        "python3 -c \"import os; print(os.path.join('",
        map_file_pkg_path, 
        "', 'config', '", 
        map_file_name_config, "'))\""
    ])

    # Define the map_server Node
    map_server_action = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True}, 
                    {'yaml_filename': map_file_path}]
    )

    # LogInfo action
    message_info_action = LogInfo(
        msg=["Generated map of: ", map_file_name_config]
    )

    life_cycle_manager_action = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )            

    # RViz configuration file path
    rviz_config_file = os.path.join(get_package_share_directory('map_server'), 'rviz_config', 'map_server_rviz.rviz')

    # Define the RViz Node
    rviz_action = Node(
        package='rviz2',
        executable='rviz2',
        name='map_server_rviz_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_file]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        map_file_name_action,
        message_info_action,
        map_server_action,
        rviz_action,
        life_cycle_manager_action,
    ])
