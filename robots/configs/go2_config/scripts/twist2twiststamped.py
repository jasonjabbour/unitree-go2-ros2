#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistStampedRepublisher(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')

        # Subscribe to /cmd_vel (Twist)
        self.sub_twist = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.callback_twist,
            10
        )

        # Publish to /cmd_vel_stamped (TwistStamped)
        self.pub_twist_stamped = self.create_publisher(
            TwistStamped,
            '/cmd_vel_stamped',
            10
        )

    def callback_twist(self, msg):
        # Convert Twist -> TwistStamped
        msg_out = TwistStamped()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'base_link'  # or any frame you want
        msg_out.twist = msg

        self.pub_twist_stamped.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = TwistStampedRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()