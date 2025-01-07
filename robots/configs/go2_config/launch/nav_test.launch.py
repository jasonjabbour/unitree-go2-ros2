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

    container = ComposableNodeContainer(
        name='depthimage_to_laserscan_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[

            # Convert Depth Image to Laserscan
            ComposableNode(
                # namespace="robotperf/benchmark",
                package='pointcloud_to_laserscan',
                plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
                name='pointcloud_to_laserscan_node',
                remappings=[
                    ('cloud_in', '/velodyne_points'),
                ], 
                parameters=[ {'scan_time': 0.000000001} 
                ]
                
            ), 

        ],
        output='screen',
    )

    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[map_server_params_file, {'yaml_filename': map_yaml_file, 'use_sim_time': True}]
    )

    # AMCL Node
    amcl = Node(
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
            ('map', '/map')
        ]
    )

    # # Static transform publisher for map -> odom
    # static_tf_map_to_odom = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_map_to_odom",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    #     parameters=[{'use_sim_time': True}]
    # )

    # # Static transform publisher for odom -> base_link
    # static_tf_odom_to_base_link = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_transform_odom_to_base_link",
    #     arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    #     parameters=[{'use_sim_time': True}]
    # )

    # # Planner Server Node (Global Planner)
    # planner_server = Node(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},  # Ensure use_sim_time
    #         os.path.join(package_share_directory, 'config', 'navigation', 'global_planner_params.yaml')
    #     ],
    #     remappings=[
    #         ('plan', '/global_plan')  # Output topic for the computed path
    #     ]
    # )

    # # Controller Server Node (Local Planner)
    # controller_server = Node(
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},  # Ensure use_sim_time is enabled
    #         os.path.join(package_share_directory, 'config', 'navigation', 'local_planner_params.yaml')  # Updated config file
    #     ],
    #     remappings=[
    #         ('/cmd_vel', '/cmd_vel'),  # Output velocity commands
    #         ('/plan', '/global_plan')  # Global path to follow
    #     ]
    # )


    # Lifecycle Manager to automatically manage lifecycle states
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,        # Enable simulated time
            'autostart': True,           # Automatically start lifecycle nodes
            'node_names': ['map_server', 'amcl'] #, 'planner_server', 'controller_server']  # List of nodes to manage
        }]
    )

    # Add nodes to the launch description
    ld.add_action(container)
    ld.add_action(map_server)
    ld.add_action(amcl)
    # ld.add_action(static_tf_map_to_odom)
    # ld.add_action(static_tf_odom_to_base_link)
    # ld.add_action(planner_server)
    # ld.add_action(controller_server)
    ld.add_action(lifecycle_manager)


    return ld