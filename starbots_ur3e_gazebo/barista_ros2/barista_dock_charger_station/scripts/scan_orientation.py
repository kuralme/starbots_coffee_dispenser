#! /usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
import numpy as np



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('scan_orientation')
        self.subscription = self.create_subscription(
            LaserScan,
            '/barista_1/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.ranges = msg.ranges
        np_ranges = np.array(self.ranges)
        ranges_length = len(np_ranges)
        # Make sure to verify the lidar orientation. 
        # Run this node and check the front, back left and right
        # this is for rplidar a1
        back_range = np_ranges[0]
        front_range = np_ranges[500]
        right_range = np_ranges[int(ranges_length/4)]
        left_range = np_ranges[int(3*ranges_length/4)]
        print(front_range)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




