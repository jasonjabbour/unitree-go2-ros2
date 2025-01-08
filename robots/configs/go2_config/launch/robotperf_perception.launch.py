
import json
import os
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

# These are provided as environment variables for CI, you can manually hardcord them for other uses
rosbag = 'end_to_end/rosbag2_end_to_end_1m' #os.environ.get('ROSBAG') # Example: 'perception/r2b_cafe'
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
    """Generate launch description for benchmarking depthimage_to_laserscan DepthImageToLaserScanROS"""

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestEnd2EndPerception.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('velodyne_points', 'data_loader/velodyne_points'),
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
                'sensor_msgs/msg/PointCloud2',
            ],
            'qos': {'reliability': 'best_effort', 'durability': 'transient_local'},
        }],
        remappings=[('buffer/input0', 'data_loader/velodyne_points'), #subbing to
                    ('input0', 'benchmark/velodyne_points'), #publishing to
                    ],                   
    )

    # Convert PointCloud to Laserscan (node of interest)
    laserscan_node = ComposableNode(
            namespace='robotperf/benchmark',
            package='pointcloud_to_laserscan',
            plugin='pointcloud_to_laserscan::PointCloudToLaserScanNode',
            name='pointcloud_to_laserscan',
            remappings=[
                ('cloud_in', '/robotperf/benchmark/velodyne_points'),
                ('scan', '/robotperf/benchmark/scan')
            ], 
            parameters=[ {'scan_time': 0.000000001, 
                        'qos': {'reliability': 'best_effort', 'durability': 'transient_local'}}
            ]
    )



    monitor_node = ComposableNode(
        name='MonitorNode',
        namespace=TestEnd2EndPerception.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::MonitorNode',
        parameters=[{
            'monitor_data_format': 'sensor_msgs/msg/LaserScan',
            # 'monitor_power_data_format': 'power_msgs/msg/Power',
            'qos': {'reliability': 'best_effort', 'durability': 'transient_local'}, 
            'qos_type': 'sensor',

        }],
        remappings=[
            ('output', '/robotperf/benchmark/scan')],
    )

    if OPTION == 'with_monitor_node':
        composable_node_descriptions_option=[
            data_loader_node,
            playback_node,
            laserscan_node,
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
        return [composable_node_container, power_container]
    else:
        return [composable_node_container]


class TestEnd2EndPerception(ROS2BenchmarkTest):
    """Performance test for depthimage_to_laserscan DepthImageToLaserScanROS."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='depthimage_to_laserscan::DepthImageToLaserScanROS Benchmark',
        input_data_path=ROSBAG_PATH,
        benchmark_namespace='robotperf', 
        # How long to wait to load in recording node times out
        start_recording_service_timeout_sec=150, 
        start_recording_service_future_timeout_sec=150,
        # The number of frames to be buffered
        playback_message_buffer_size=30,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=30.0,
        publisher_lower_frequency=30.0,
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
                str_out += "| ros2_benchmark | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | **{:.2f}** ms | {:.2f} % |\n".format(
                    mean_latency, rms_latency, max_latency, min_latency, (frames_missed/frames_sent)*100)
            print(str_out)
     
def generate_test_description():
    return TestEnd2EndPerception.generate_test_description_with_nsys(launch_setup)