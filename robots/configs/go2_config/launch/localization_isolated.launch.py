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

    # Make sure /bond topic is running from ROS Bag
    # $ ros2 bag play rosbag2_end_to_end_1 --loop --topics /bond

    # AMCL Node
    amcl = Node(
        namespace='robotperf/benchmark',
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            nav_params_file
        ],
        remappings=[
            ('scan', '/scan'),
            ('map', '/map'), 
            ('initialpose', '/initialpose'), 
            ('clock', '/clock'), 
            # ('bond', '/bond'),
            ('tf', '/tf'), 
            ('tf_static', '/tf_static'), 
        ]
    )



    # Lifecycle Manager to automatically manage lifecycle states
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        namespace='robotperf/benchmark',
        parameters=[{
            'use_sim_time': True,        # Enable simulated time
            'autostart': True,           # Automatically start lifecycle nodes
            'node_names': ['amcl'] #, 'planner_server', 'controller_server']  # List of nodes to manage
        }]
    )


    return LaunchDescription(
        [
            amcl, 
            lifecycle_manager,
        ]
    )
