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
import time

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    # Paths for the map files
    package_share_directory = get_package_share_directory('go2_config')
    map_yaml_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'square_map.yaml')
    map_server_params_file = os.path.join(package_share_directory, 'maps_manual', 'square_map', 'map_server.yaml')
    nav_params_file = os.path.join(package_share_directory, 'config', 'navigation', 'nav2_params.yaml')

    # Paths for configuration files
    bt_xml_file = os.path.join(package_share_directory, 'config', 'navigation', 'navigate_w_replanning_and_recovery.xml')

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "/config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_VLP.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/square.world")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )


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

    # global_costmap_node = Node(
    #     package='nav2_costmap_2d',
    #     executable='costmap_2d_node',
    #     name='global_costmap',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},  # Synchronize with simulation
    #         nav_params_file
    #     ],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
    #     ]
    # )


    # local_costmap_node = Node(
    #     package='nav2_costmap_2d',
    #     executable='costmap_2d_node',
    #     name='local_costmap',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},  # Synchronize with simulation
    #         nav_params_file 
    #     ],
    #     remappings=[
    #         ('/tf', 'tf'),
    #         ('/tf_static', 'tf_static')
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
            'node_names': ['map_server',
                           'amcl'] #, 'planner_server', 'controller_server', 'bt_navigator', 'behavior_server']  # List of nodes to manage
        }]
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
            "close_loop_odom": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            bringup_ld,
            gazebo_ld,
            container, 
            map_server, 
            amcl, 
            # global_costmap_node,  
            # local_costmap_node, 
            # planner_server, 
            # controller_server, 
            # bt_navigator,
            # behavior_server,
            lifecycle_manager,
        ]
    )




 # # Planner Server Node (Global Planner)
    # planner_server = Node(
    #     package='nav2_planner',
    #     executable='planner_server',
    #     name='planner_server',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},  # Ensure use_sim_time
    #         nav_params_file
    #     ],
    #     remappings=[
    #         ('plan', '/global_plan')  # Output topic for the computed path
    #     ]
    # )

    # # Controller Server Node (Local Controller)
    # controller_server = Node(
    #     package='nav2_controller',
    #     executable='controller_server',
    #     name='controller_server',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True},  # Ensure use_sim_time
    #         nav_params_file
    #     ]
    # )


    # # BT Navigator Node
    # bt_navigator = Node(
    #     package='nav2_bt_navigator',
    #     executable='bt_navigator',
    #     name='bt_navigator',
    #     output='screen',
    #     parameters=[
    #         {'use_sim_time': True, 
    #         'default_nav_through_poses_bt_xml': bt_xml_file},  # Ensure use_sim_time
    #         nav_params_file,
    #     ]
    # )

    # behavior_server = Node(
    #     package="nav2_behaviors",
    #     executable="behavior_server",
    #     name="behavior_server",
    #     output="screen",
    #     parameters=[
    #         nav_params_file
    #     ]
    # )