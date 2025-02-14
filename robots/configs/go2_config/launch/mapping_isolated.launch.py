import os

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    # Paths for the map files
    package_share_directory = get_package_share_directory('go2_config')
    nav_params_file = os.path.join(package_share_directory, 'config', 'navigation', 'nav2_params.yaml')
    map_yaml_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'square_map.yaml')
    map_server_params_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'map_server.yaml')

    # Paths for configuration files
    bt_xml_file = os.path.join(package_share_directory, 'config', 'navigation', 'navigate_w_replanning_and_recovery.xml')

    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_server_params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': True}]
    )


    # Controller Server Node (Local Controller)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},  # Ensure use_sim_time
            nav_params_file
        ]
    )

    # BT Navigator Node
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': True, 
            'default_nav_through_poses_bt_xml': bt_xml_file},  # Ensure use_sim_time
            nav_params_file,
        ]
    )

    # Lifecycle Manager to automatically manage lifecycle states
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,        # Enable simulated time
            'autostart': True,           # Automatically start lifecycle nodes
            'node_names': ['map_server', 
                           'controller_server', 
                           'bt_navigator'] 
        }]
    )


    return LaunchDescription(
        [
            map_server, 
            controller_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )
