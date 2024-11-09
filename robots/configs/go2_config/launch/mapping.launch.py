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

    # # SLAM Toolbox node
    # slam_toolbox_node = Node(
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},
    #         {'slam_params_file': os.path.join(
    #             get_package_share_directory('go2_config'), 'config', 'autonomy' ,'mapper_params_online_async.yaml')},
    #     ],
    #     remappings=[
    #         ('scan', '/scan'),  # Use the /scan topic from the LaserScan node
    #     ]
    # )

#    # Cartographer node with default config and parameter overrides
#     cartographer_node = Node(
#         package='cartographer_ros',
#         executable='cartographer_node',
#         name='cartographer_node',
#         output='screen',
#         parameters=[
#             {'use_sim_time': True},  # Use simulation time if needed
#             {'tracking_frame': 'base_link'},  # Set the tracking frame
#             {'published_frame': 'base_link'},  # Set the frame that is published as the robot's current position
#             {'odom_frame': 'odom'},  # Use odometry frame
#             {'provide_odom_frame': False},  # If you're using an external odometry source, set this to False
#             {'use_odometry': True},  # Use odometry
#             {'use_imu_data': True},   # Use IMU data if available
#             {'num_multi_echo_laser_scans': 0}, 
#             {'num_point_clouds': 1}, 
#             {'num_laser_scans': 1}
#         ],
#         arguments=[
#             '-configuration_directory', '/opt/ros/humble/share/cartographer_ros/configuration_files/',
#             '-configuration_basename', 'backpack_2d.lua'
#         ],
#         remappings=[
#             ('scan', '/scan'),  # Remap the laser scan topic if needed
#             ('imu', '/imu/data'),  # Remap to your IMU topic
#             ('odom', '/odom/ground_truth'),  # Remap to your odometry topic
#             ('points2', 'velodyne_points')
#         ]
#     )
    
   # Cartographer node with default config and parameter overrides
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
        ],
        arguments=[
            '-configuration_directory', os.path.join(get_package_share_directory('go2_config'), 'config', 'autonomy'),
            '-configuration_basename', 'custom_cartographer_2d.lua'
        ],
        remappings=[
            ('scan', '/scan'),  # Remap the laser scan topic if needed
            ('imu', '/imu/data'),  # Remap to your IMU topic
            ('odom', '/odom/ground_truth'),  # Remap to your odometry topic
            ('points2', 'velodyne_points')
        ]
    )
    
    # OccupancyGrid node to generate the map
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},  # Use simulation time if needed
        ],
        arguments=[
            '-resolution', '0.05'  # Set the map resolution, adjust as needed
        ]
    )


    
    # Add the components to the launch description
    ld.add_action(container)
    # ld.add_action(slam_toolbox_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)


    return ld



