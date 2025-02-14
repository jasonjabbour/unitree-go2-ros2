#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
import time

class SimClockPublisher(Node):
    def __init__(self):
        super().__init__('sim_clock_publisher')
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.timer = self.create_timer(0.01, self.publish_time)  # Publish at 100Hz

        self.sim_time = 0.0  # Start time
        self.start_time = time.time()  # Real-world start time

    def publish_time(self):
        elapsed_time = time.time() - self.start_time
        self.sim_time = elapsed_time  # Simulated time matches real elapsed time

        clock_msg = Clock()
        clock_msg.clock.sec = int(self.sim_time)
        clock_msg.clock.nanosec = int((self.sim_time - int(self.sim_time)) * 1e9)

        self.clock_pub.publish(clock_msg)
        self.get_logger().info(f'Publishing Simulated Time: {clock_msg.clock.sec}.{clock_msg.clock.nanosec}')

def main(args=None):
    rclpy.init(args=args)
    node = SimClockPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()