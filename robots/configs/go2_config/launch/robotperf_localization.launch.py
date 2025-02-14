
import json
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

# These are provided as environment variables for CI, you can manually hardcord them for other uses
rosbag = 'end_to_end/rosbag2_end_to_end_1' #os.environ.get('ROSBAG') # Example: 'perception/r2b_cafe'
package = 'end_to_end_localization' #os.environ.get('PACKAGE') # Example: 'a6_depthimage_to_laserscan'
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

    package_share_directory = get_package_share_directory('go2_config')
    nav_params_file = os.path.join(package_share_directory, 'config', 'navigation', 'nav2_params.yaml')


    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        # namespace=TestEnd2EndLocalization.generate_namespace(),
        namespace='robotperf/benchmark',
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('map', 'data_loader/map'),
                    ('scan', 'data_loader/scan'),
                    # ('bond', 'data_loader/bond'),
                    ('initialpose', 'data_loader/initialpose'),
                    ('tf', 'data_loader/tf'),
                    ('tf_static', 'data_loader/tf_static'),
                    ('clock', 'data_loader/clock'),
                    ]                    
    )

    # Publish input messages by using the messages stored in the buffer
    playback_node = ComposableNode(
        name='PlaybackNode',
        # namespace=TestEnd2EndLocalization.generate_namespace(),
        namespace='robotperf/benchmark',
        package='ros2_benchmark',
        plugin='ros2_benchmark::PlaybackNode',
        parameters=[{
            'data_formats': [
                'nav_msgs/msg/OccupancyGrid',
                'sensor_msgs/msg/LaserScan',
                # 'bond/msg/Status',
                'geometry_msgs/msg/PoseWithCovarianceStamped',
                'tf2_msgs/msg/TFMessage',  
                'tf2_msgs/msg/TFMessage',  
                'rosgraph_msgs/msg/Clock',  
            ],
            'qos': {'reliability': 'best_effort', 'durability': 'transient_local'},
        }],
        remappings=[('buffer/input0', 'data_loader/map'), #subbing to
                    ('input0', '/map'), #publishing to
                    ('buffer/input1', 'data_loader/scan'), #subbing to
                    ('input1', '/scan'), #publishing to
                    # ('buffer/input2', 'data_loader/bond'), #subbing to
                    # ('input2', '/bond'), #publishing to
                    ('buffer/input2', 'data_loader/initialpose'), #subbing to
                    ('input2', '/initialpose'), #publishing to
                    ('buffer/input3', 'data_loader/tf'), #subbing to
                    ('input3', '/tf'), #publishing to
                    ('buffer/input4', 'data_loader/tf_static'), #subbing to
                    ('input4', '/tf_static'), #publishing to
                    ('buffer/input5', 'data_loader/clock'), #subbing to
                    ('input5', '/clock'), #publishing to
                    ],                   
    )

    # AMCL Node
    amcl_node = Node(
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

    monitor_node = ComposableNode(
        name='MonitorNode',
        # namespace=TestEnd2EndLocalization.generate_namespace(),
        namespace='robotperf/benchmark',
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'geometry_msgs/msg/PoseWithCovarianceStamped',
            # 'monitor_power_data_format': 'power_msgs/msg/Power',
            # 'qos': {'reliability': 'best_effort', 'durability': 'transient_local'}, 
            # 'qos_type': 'sensor',

        }],
        remappings=[
            ('output', '/robotperf/benchmark/amcl_pose')],
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
            'autostart': False,           # Automatically start lifecycle nodes
            'node_names': ['amcl'] 
        }]
    )

    # Event handler to start Lifecycle Manager only after DataLoaderNode exits successfully
    start_lifecycle_after_data_loader = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=amcl_node,
            on_exit=[lifecycle_manager]
        )
    )

    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            monitor_node          
        ]
    else:
        composable_node_descriptions_option=[
            # data_loader_node,
            # playback_node,

        ]

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestEnd2EndLocalization.generate_namespace(),
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
        return [composable_node_container, amcl_node, lifecycle_manager, power_container]
    else:
        return [composable_node_container, amcl_node, lifecycle_manager, start_lifecycle_after_data_loader]


class TestEnd2EndLocalization(ROS2BenchmarkTest):
    """Performance test for depthimage_to_laserscan DepthImageToLaserScanROS."""

    # # Custom configurations
    # config = ROS2BenchmarkConfig(
    #     benchmark_name='depthimage_to_laserscan::DepthImageToLaserScanROS Benchmark',
    #     input_data_path=ROSBAG_PATH,
    #     benchmark_namespace='robotperf/benchmark', 
    #     # How long to wait to load in recording node times out
    #     start_recording_service_timeout_sec=30, #150, 
    #     start_recording_service_future_timeout_sec=30, #150,
    #     # The number of frames to be buffered
    #     playback_message_buffer_size=3, #30,
    #     # Upper and lower bounds of peak throughput search window
    #     publisher_upper_frequency=30.0,
    #     publisher_lower_frequency=30.0,
    #     custom_report_info={'data_resolution': IMAGE_RESOLUTION},
    #     option = OPTION,
    #     session_name = SESSION_NAME,
    #     add_power = POWER, 
    #     log_folder='/tmp/benchmark_ws/results'
    # )
    
    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='End2End Localization Benchmark',
        input_data_path=ROSBAG_PATH,
        benchmark_namespace='robotperf/benchmark', 
        # Amount of time to wait for a service client to be created
        setup_service_client_timeout_sec=100,
        # Amount of time for a service to complete before timing out
        start_recording_service_timeout_sec=125,
        start_monitoring_service_timeout_sec=200,
        # Amount of time to wait for a service future to return
        default_service_future_timeout_sec= 45,
        set_data_service_future_timeout_sec=100,
        start_recording_service_future_timeout_sec=150,
        play_messages_service_future_timeout_sec=150,
        # The number of frames to be buffered
        playback_message_buffer_size=1, #30,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=30.0,
        publisher_lower_frequency=30.0,
        custom_report_info={'data_resolution': IMAGE_RESOLUTION},
        option = OPTION,
        session_name = SESSION_NAME,
        add_power = POWER, 
        log_folder='/tmp/benchmark_ws/results', 
        # Enable publishing TF messages when setting data in a data loader node
        publish_tf_messages_in_set_data=True, 
        publish_tf_static_messages_in_set_data=True, 
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
                str_out += "| ros2_benchmark | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | {:.2f} % |\n".format(
                    mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100)
            print(str_out)
     
def generate_test_description():
    return TestEnd2EndLocalization.generate_test_description_with_nsys(launch_setup)