import os
import xacro
import launch_ros
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler
)
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.event_handlers.on_execution_complete import OnExecutionComplete
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration


def generate_launch_description():

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_config"
    ).find("champ_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="champ_description"
    ).find("champ_description")

    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")

    # default_rviz_path = os.path.join(descr_pkg_share, "rviz/urdf_viewer.rviz")
    default_rviz_path = os.path.join(descr_pkg_share, "rviz/navigation_viewer.rviz")
    default_model_path = os.path.join(descr_pkg_share, "urdf/champ.urdf.xacro")


    declare_description_path = DeclareLaunchArgument(
        name="description_path",
        default_value=default_model_path,
        description="Absolute path to robot urdf file",
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_joints_map_path = DeclareLaunchArgument(
        name="joints_map_path",
        default_value=joints_config,
        description="Absolute path to joints map file",
    )

    declare_links_map_path = DeclareLaunchArgument(
        name="links_map_path",
        default_value=links_config,
        description="Absolute path to links map file",
    )

    declare_gait_config_path = DeclareLaunchArgument(
        name="gait_config_path",
        default_value=gait_config,
        description="Absolute path to gait config file",
    )


    declare_gazebo = DeclareLaunchArgument(
        "gazebo", default_value="true", description="If in gazebo"
    )

    declare_joint_controller_topic = DeclareLaunchArgument(
        "joint_controller_topic",
        default_value="joint_group_effort_controller/joint_trajectory",
        description="Joint controller topic",
    )

    declare_publish_joint_control = DeclareLaunchArgument(
        "publish_joint_control",
        default_value="true",
        description="Publish joint control",
    )

    declare_publish_joint_states = DeclareLaunchArgument(
        "publish_joint_states",
        default_value="true",
        description="Publish joint states",
    )

    declare_publish_foot_contacts = DeclareLaunchArgument(
        "publish_foot_contacts",
        default_value="true",
        description="Publish foot contacts",
    )


    # description_ld = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(
    #             get_package_share_directory("champ_description"),
    #             "launch",
    #             "description.launch.py",
    #         )
    #     ),
    #     launch_arguments={
    #         "use_sim_time": LaunchConfiguration("use_sim_time"),
    #         "description_path": LaunchConfiguration("description_path"),
    #         "gazebo": LaunchConfiguration("gazebo")
    #     }.items(),
    # )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"gazebo": LaunchConfiguration("gazebo")},
            {"publish_joint_states": LaunchConfiguration("publish_joint_states")},
            {"publish_joint_control": LaunchConfiguration("publish_joint_control")},
            {"publish_foot_contacts": LaunchConfiguration("publish_foot_contacts")},
            {"joint_controller_topic": LaunchConfiguration("joint_controller_topic")},
            {"urdf": Command(['xacro ', LaunchConfiguration('description_path')])},
            LaunchConfiguration('joints_map_path'),
            LaunchConfiguration('links_map_path'),
            LaunchConfiguration('gait_config_path'),
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

   
    return LaunchDescription(
        [
            declare_use_sim_time,
            declare_description_path,
            declare_joints_map_path, 
            declare_links_map_path,
            declare_gait_config_path,
            declare_gazebo,
            declare_joint_controller_topic,
            declare_publish_joint_control,
            declare_publish_joint_states,
            declare_publish_foot_contacts,
            # description_ld,
            quadruped_controller_node
        ]
    )

