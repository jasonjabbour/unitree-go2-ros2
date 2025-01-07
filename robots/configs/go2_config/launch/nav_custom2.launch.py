import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Paths for the map files
    package_share_directory = get_package_share_directory('go2_config')
    map_yaml_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'square_map.yaml')
    map_server_params_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'map_server.yaml')
    nav_params_file = os.path.join(package_share_directory, 'config', 'navigation', 'nav2_params.yaml')


    ld = LaunchDescription()




    # Lifecycle Manager to automatically manage lifecycle states
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,        # Enable simulated time
            'autostart': True,           # Automatically start lifecycle nodes
            'node_names': [] 
        }]
    )

    # Add nodes to the launch description
    ld.add_action(costmap)
    ld.add_action(lifecycle_manager)


    return ld