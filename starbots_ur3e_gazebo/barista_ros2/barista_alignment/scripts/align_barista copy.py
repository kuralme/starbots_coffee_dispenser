#! /usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from barista_alignment.action import AlignBarista
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class Aligner(Node):
    def __init__(self):
        super().__init__('barista_aligner')

        self.callback_group = ReentrantCallbackGroup()

        self.ranges = []
        self.index_a = 520
        self.index_b = 480
        self.index_x = 350
        self.index_y = 150
        self.desired_range_a = 0.85
        self.desired_range_b = 0.85
        self.desired_right_range = 0.40
        self.thresh = 0.02
        self.P = 0.9
        self.D = 0.05
        self.front_range_ix = 500
        self.scan_offset = 0.103

        self.angular_controller = Controller(P=self.P, D=self.D)

        self.aligning = False

        self.scan_sub_ = self.create_subscription(LaserScan, 'barista_1/scan', self.scan_callback, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.callback_group
        )

        self._action_server = ActionServer(self, AlignBarista, 'align_barista', self.align, callback_group=self.callback_group)

        # self.timer = self.create_timer(0.1, self.align, callback_group=self.callback_group)

        self.vel_pub_ = self.create_publisher(Twist, 'barista_1/cmd_vel', 10, callback_group=self.callback_group)
        self.vel_cmd = Twist()

    def scan_callback(self, msg):
        self.ranges = np.array(msg.ranges)
        self.ranges_array_length = len(self.ranges)
        self.distance_a = self.ranges[515:self.index_a]
        self.distance_b = self.ranges[self.index_b:485]
        self.front_range = self.ranges[self.front_range_ix]
        self.front_vision = self.ranges[400:600]
        self.right_range_ix = int(self.ranges_array_length/4)
        self.right_range = self.ranges[self.right_range_ix]
        self.distance_x = self.ranges[345:self.index_x]
        self.distance_y = self.ranges[self.index_y:155]   
        # print(self.distance_x)
        # print(self.distance_y)
        # print(np.nanmin(self.distance_y))
        self.back_range = self.ranges[0]
        self.distance_1 = self.ranges[self.front_range_ix + 50]
        self.distance_2 = self.ranges[self.front_range_ix - 50]


    def stop(self):
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.z = 0.0
        self.vel_pub_.publish(self.vel_cmd)

    def rotate(self, direction, w=0.5, x=0.0):
        if direction == "left":
            self.vel_cmd.angular.z = w
            self.vel_cmd.linear.x = x
        elif direction == "right":
            self.vel_cmd.angular.z = -w
            self.vel_cmd.linear.x = x
        elif direction is None:
            self.vel_cmd.angular.z = w
            self.vel_cmd.linear.x = x
        self.vel_pub_.publish(self.vel_cmd)

    def align(self, goal_handle):
        if not self.aligning:
            self.aligning = True
            feedback_msg = AlignBarista.Feedback()

            while self.aligning:
                perform_control = True
                setpoint = 0
                self.angular_controller.setPoint(setpoint)
                rate = self.create_rate(10)

                while perform_control:
                    front_vision_transformed = np.sqrt(np.square(self.scan_offset)+np.square(self.front_vision))
                    feedback_msg.front = float(np.nanmin(front_vision_transformed))
                    goal_handle.publish_feedback(feedback_msg)
                    control_variable = np.mean(self.distance_a[np.isfinite(self.distance_a)]) - np.mean(self.distance_b[np.isfinite(self.distance_b)])
                    vel_w = self.angular_controller.update(control_variable)
                    perform_control = np.nanmin(front_vision_transformed) > 0.40
                    self.rotate(direction=None, w=vel_w, x=0.05)
                    rate.sleep()

                self.stop()
                while self.right_range > np.nanmin(self.ranges) + 0.002:
                    self.rotate(direction="left", w=0.3)
                self.stop()
                perform_control = True
                setpoint = 0
                self.angular_controller.setPoint(setpoint)

                while perform_control:
                    feedback_msg.right = float(self.right_range)
                    goal_handle.publish_feedback(feedback_msg)
                    control_variable = np.mean(self.distance_x[np.isfinite(self.distance_x)]) - np.mean(self.distance_y[np.isfinite(self.distance_y)]) + self.desired_right_range - self.right_range
                    vel_w = self.angular_controller.update(control_variable)
                    perform_control = (self.back_range > 0.6)
                    self.rotate(direction=None, w=vel_w, x=-0.05)
                    rate.sleep()

                self.stop()
                self.aligning = False

            print("Barista aligned")
            goal_handle.succeed()
            result = AlignBarista.Result()
            result.complete = True
            return result    
        else:
            pass 

def main(args=None):
    rclpy.init(args=args)

    try:
        barista_aligner = Aligner() 
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(barista_aligner)

        try:
            executor.spin()

        finally:
            executor.shutdown()
            barista_aligner.destroy_node()

    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
