
import json
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import launch_ros
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler
)
from launch import LaunchDescription


from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

# These are provided as environment variables for CI, you can manually hardcord them for other uses
rosbag = 'end_to_end/rosbag2_cmd_vel_stamped' #os.environ.get('ROSBAG') # Example: 'perception/r2b_cafe'
package = 'end_to_end_perception' #os.environ.get('PACKAGE') # Example: 'a6_depthimage_to_laserscan'
type = 'black' #os.environ.get('TYPE') # Example: 'grey'
metric = 'latency' #os.environ.get('METRIC') # Example: 'latency'

POWER_LIB = os.environ.get('POWER_LIB')
IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = '/tmp/benchmark_ws/src/rosbags/' + rosbag # '/home/amf/benchmark_ws/src/rosbags/perception/image3' # NOTE: hardcoded, modify accordingly
SESSION_NAME = package
if type == "grey":
    OPTION = 'without_monitor_node'
else:
    OPTION = 'with_monitor_node'
if metric == "power":
    POWER = "on" # by default "off"
else:
    POWER = "off"

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description """


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

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestEnd2EndPerception.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('cmd_vel_stamped', 'data_loader/cmd_vel'),
                    ]                    
    )

    # Publish input messages by using the messages stored in the buffer
    playback_node = ComposableNode(
        name='PlaybackNode',
        namespace=TestEnd2EndPerception.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::PlaybackNode',
        parameters=[{
            'data_formats': [
                'geometry_msgs/msg/TwistStamped',
            ],
            'qos': {'reliability': 'reliable', 'durability': 'transient_local'},
            # 'qos': {'reliability': 'best_effort', 'durability': 'transient_local'},
            'use_sim_time': True,
        }],
        remappings=[('buffer/input0', 'data_loader/cmd_vel'), #subbing to
                    ('input0', '/robotperf/benchmark/cmd_vel/smooth'), #publishing to
                    ],                   
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        namespace='robotperf/benchmark',
        parameters=[
            {"use_sim_time": True},
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
        remappings=[("/cmd_vel/smooth", "playback/cmd_vel")],
    )

   

    # # Convert PointCloud to Laserscan (node of interest)
    # laserscan_node = ComposableNode(
    #         namespace='robotperf/benchmark',
    #         package='pointcloud_to_laserscan',
    #         plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
    #         name='pointcloud_to_laserscan',
    #         remappings=[
    #             ('cloud_in', '/robotperf/benchmark/velodyne_points'),
    #             ('scan', '/robotperf/benchmark/scan')
    #         ], 
    #         parameters=[ {'scan_time': 0.000000001, 
    #                     'qos': {'reliability': 'best_effort', 'durability': 'transient_local'}}
    #         ]
    # )



    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=TestEnd2EndPerception.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'trajectory_msgs/msg/JointTrajectory',
            'qos': {'reliability': 'reliable', 'durability': 'transient_local'}, 
            'qos_type': 'sensor',
            'use_sim_time': True,
        }],
        remappings=[
            ('output', '/robotperf/benchmark/joint_group_effort_controller/joint_trajectory')],
    )




    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            monitor_node           
        ]
    else:
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,

        ]

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestEnd2EndPerception.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=composable_node_descriptions_option,
        output='screen'
    )

    if POWER == "on":
        power_container = ComposableNodeContainer(
            name="power_container",
            namespace="robotcore/power",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="robotcore-power",
                    namespace="robotcore/power",
                    plugin="robotcore::power::PowerComponent",
                    name="power_component",
                    parameters=[
                        {"publish_rate": 20.0},
                        {"power_lib": POWER_LIB}
                    ],
                ),
                
            ],
            output="screen",
        )
        return [composable_node_container, 
                power_container, 
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
                quadruped_controller_node]
    else:
        return [composable_node_container,
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
                quadruped_controller_node]


class TestEnd2EndPerception(ROS2BenchmarkTest):
    """Performance test for depthimage_to_laserscan DepthImageToLaserScanROS."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='depthimage_to_laserscan::DepthImageToLaserScanROS Benchmark',
        input_data_path=ROSBAG_PATH,
        benchmark_namespace='robotperf', 
        revise_timestamps_as_message_ids=False,
        # benchmark_mode="TIMELINE",
        # record_data_timeline=True,
        # load_data_in_real_time=True,
        # revise_timestamps_as_message_ids=True, 
        # How long to wait to load in recording node times out
        start_recording_service_timeout_sec=100, 
        start_recording_service_future_timeout_sec=100,
        # The number of frames to be buffered
        playback_message_buffer_size=50,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=30,
        publisher_lower_frequency=30,
        custom_report_info={'data_resolution': IMAGE_RESOLUTION},
        option = OPTION,
        session_name = SESSION_NAME,
        add_power = POWER, 
        log_folder='/tmp/benchmark_ws/results'
    )
    
    def test_benchmark(self):
        json_file_path = self.run_benchmark()

        # Copy the JSON file to the "/tmp/json" file
        # NOTE: this will be then used by the CI to post-process and analyze results
        os.system("cp " + json_file_path + " /tmp/json")

        if self.config.option == 'with_monitor_node':
            # Open the file and load the JSON content into a Python dictionary
            with open(json_file_path, 'r') as f:
                data = json.load(f)
            # Extract the desired fields
            mean_latency = data.get("BasicPerformanceMetrics.MEAN_LATENCY")
            max_latency = data.get("BasicPerformanceMetrics.MAX_LATENCY")
            min_latency = data.get("BasicPerformanceMetrics.MIN_LATENCY")
            rms_latency = data.get("BasicPerformanceMetrics.RMS_LATENCY")
            frames_sent = int(data.get("BasicPerformanceMetrics.NUM_FRAMES_SENT"))
            frames_missed = int(data.get("BasicPerformanceMetrics.NUM_MISSED_FRAMES"))  
            if self.config.add_power == "on":
                average_power = data.get("BasicPerformanceMetrics.AVERAGE_POWER")   
                str_out =  "|     | Benchmark Mean | Benchmark RMS | Benchmark Max  | Benchmark Min | Lost Messages | Average Power |\n"
                str_out += "| --- | -------------- | ------------- | -------------- | ------------- | --------------| ------------------|\n"
                str_out += "| ros2_benchmark | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | {:.2f} % | **{:.2f}** W |\n".format(
                mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100, average_power)         
            else:
                str_out =  "|     | Benchmark Mean | Benchmark RMS | Benchmark Max  | Benchmark Min | Lost Messages |\n"
                str_out += "| --- | -------------- | ------------- | -------------- | ------------- | --------------|\n"
                str_out += "| ros2_benchmark | **{}** ms | **{}** ms | **{}** ms | **{}** ms | {} % |\n".format(
                    mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100)
            print(str_out)
     
def generate_test_description():
    return TestEnd2EndPerception.generate_test_description_with_nsys(launch_setup)