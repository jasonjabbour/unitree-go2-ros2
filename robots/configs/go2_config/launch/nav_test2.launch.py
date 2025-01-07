
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths for configuration files
    package_share_directory = get_package_share_directory('go2_config')
    nav_config = os.path.join(package_share_directory, 'config', 'navigation', 'nav2_params.yaml')
    bt_xml_file = os.path.join(package_share_directory, 'config', 'navigation', 'navigate_w_replanning_and_recovery.xml')


    ld = LaunchDescription()

    # Planner Server Node (Global Planner)
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},  # Ensure use_sim_time
            nav_config
        ],
        remappings=[
            ('plan', '/global_plan')  # Output topic for the computed path
        ]
    )

    # Controller Server Node (Local Controller)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},  # Ensure use_sim_time
            nav_config
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
            nav_config,
        ]
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            nav_config
        ]
    )

    # Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Ensure use_sim_time
            'autostart': True,     # Automatically start lifecycle nodes
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server', 
            ]
        }]
    )


    # Add nodes to the launch description
    ld.add_action(planner_server)
    ld.add_action(controller_server)
    ld.add_action(bt_navigator)
    ld.add_action(lifecycle_manager)
    ld.add_action(behavior_server)


    return ld









# import os
# from launch import LaunchDescription
# from launch_ros.actions import ComposableNodeContainer
# from launch_ros.descriptions import ComposableNode
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


# def generate_launch_description():
#     # Paths for the map files
#     package_share_directory = get_package_share_directory('go2_config')
#     map_yaml_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'square_map.yaml')
#     map_server_params_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'map_server.yaml')
    

#     ld = LaunchDescription()

    
#     # Planner Server Node (Global Planner)
#     planner_server = Node(
#         package='nav2_planner',
#         executable='planner_server',
#         name='planner_server',
#         output='screen',
#         parameters=[
#             {'use_sim_time': True},  # Ensure use_sim_time
#             os.path.join(package_share_directory, 'config', 'navigation', 'global_planner_params.yaml')
#         ],
#         remappings=[
#             ('plan', '/global_plan')  # Output topic for the computed path
#         ]
#     )

#     # Lifecycle Manager to automatically manage lifecycle states
#     lifecycle_manager = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_navigation',
#         output='screen',
#         parameters=[{
#             'use_sim_time': True,        # Enable simulated time
#             'autostart': True,           # Automatically start lifecycle nodes
#             'node_names': ['planner_server']  # List of nodes to manage
#         }]
#     )

#     # Add nodes to the launch description

#     ld.add_action(planner_server)
#     ld.add_action(lifecycle_manager)


#     return ld




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
