import os
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

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

   
#    # Cartographer node with default config and parameter overrides
#     cartographer_node = Node(
#         package='cartographer_ros',
#         executable='cartographer_node',
#         name='cartographer_node',
#         output='screen',
#         parameters=[
#             {'use_sim_time': True},
#         ],
#         arguments=[
#             '-configuration_directory', os.path.join(get_package_share_directory('go2_config'), 'config', 'autonomy'),
#             '-configuration_basename', 'custom_cartographer_2d.lua'
#         ],
#         remappings=[
#             ('scan', '/scan'),  # Remap the laser scan topic if needed
#             ('imu', '/imu/data'),  # Remap to your IMU topic
#             ('odom', '/odom/ground_truth'),  # Remap to your odometry topic
#             ('points2', 'velodyne_points')
#         ]
#     )
    
#     # OccupancyGrid node to generate the map
#     occupancy_grid_node = Node(
#         package='cartographer_ros',
#         executable='cartographer_occupancy_grid_node',
#         name='occupancy_grid_node',
#         output='screen',
#         parameters=[
#             {'use_sim_time': True},  # Use simulation time if needed
#         ],
#         arguments=[
#             '-resolution', '0.05'  # Set the map resolution, adjust as needed
#         ]
#     )


    # Map Server (provides static map for AMCL and global planner)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            os.path.join(get_package_share_directory('go2_config'), 'maps_manual', 'square_map', 'map_server.yaml'),
            {'yaml_filename': os.path.join(get_package_share_directory('go2_config'), 'maps_manual', 'square_map', 'square_map.yaml')}  # Override yaml_filename dynamically
        ]
    )


    # AMCL (for localization)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{'use_sim_time': True}], 
        remappings=[
            ('scan', '/scan'),  
            ('map', '/map')  
        ]
    )


    # # Global Planner (computes path from current pose to goal)
    # planner_server = Node(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ('/plan', '/global_path'),  # Output topic for the computed path
    #         ('/goal_pose', '/navigate_goal')  # Topic for navigation goals
    #     ]
    # )


    # # Local Controller (follows global path and publishes cmd_vel)
    # controller_server = Node(
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server',
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[
    #         ('/cmd_vel', '/cmd_vel'),  # Command velocity output
    #         ('/plan', '/global_path')  # The global path to follow
    #     ]
    # )

    
    # Add the components to the launch description
    ld.add_action(container)
    # ld.add_action(cartographer_node)
    # ld.add_action(occupancy_grid_node)
    ld.add_action(map_server)
    ld.add_action(amcl)
    # ld.add_action(planner_server)
    # ld.add_action(controller_server)

    return ld



