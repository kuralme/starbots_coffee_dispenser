#! /usr/bin/env python3

import rclpy
import math
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from sensor_msgs.msg import LaserScan
from rclpy.executors import MultiThreadedExecutor
from barista_table_finder.action import AlignBarista
from barista_table_finder.action import GoUnderTable

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import ReliabilityPolicy

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import copy

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
        super().__init__('barista_table_go_under')

        # callback group for _sub and _action_server callbacks
        self.callback_group = ReentrantCallbackGroup()

        # declare parameters
        self.declare_parameter('robot_name', "barista_X")
        self.get_parameters()
        self.get_logger().warn("Robot NAME="+str(self.robot_name))

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

        # We init the positional values
        self.left_min_dist_index = None
        self.left_min_value = None
        self.right_min_dist_index = None
        self.right_min_value = None
        self.laser_length = None
        self.angle_between_left_right = None
        self.Table_Width = 0.728
        self.is_table = False
        self.MAX_DIST_FROM_TABLE = 1.5
        self.MIN_DIST_FROM_TABLE = 0.2

        self.scan_sub_ = self.create_subscription(LaserScan, '{}/scan'.format(self.robot_name), self.scan_callback, QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=self.callback_group
        )

        self.table_pos = "nothing"
        self.SCAN_BIG_DIST = 100.0
        qos_profile_publisher = QoSProfile(depth=1)
        qos_profile_publisher.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.scan_pub_ = self.create_publisher(msg_type=LaserScan,
                                                topic='{}/scan_filtered'.format(self.robot_name),
                                                qos_profile=qos_profile_publisher,
                                                callback_group=self.callback_group)


        self.action_gu_server_name = self.robot_name + '/go_under_table'
        self._action_gu_server = ActionServer(self, GoUnderTable, self.action_gu_server_name, self.go_under_table, callback_group=self.callback_group)

        self.vel_pub_ = self.create_publisher(Twist, '{}/cmd_vel'.format(self.robot_name), 10, callback_group=self.callback_group)
        self.vel_cmd = Twist()



    def get_parameters(self):
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value

    def scan_callback(self, msg):


        # We publish the filtered data hat we are going to use
        f_scan_msg = copy.deepcopy(msg)
        f_scan_msg.ranges = [self.SCAN_BIG_DIST] * len(msg.ranges)

        self.laser_length = len(f_scan_msg.ranges)

        # We get the left closest reading, x[:int(x.size/2)]
        self.left_min_dist_index = np.nanargmin(msg.ranges[:int(len(msg.ranges)/2)])
        self.left_min_value = msg.ranges[self.left_min_dist_index]
        f_scan_msg.ranges[self.left_min_dist_index] = self.left_min_value

        # We get the right closest reading, x[int(x.size/2):]
        self.right_min_dist_index = np.nanargmin(msg.ranges[int(len(msg.ranges)/2):]) + int(len(msg.ranges)/2)
        self.right_min_value = msg.ranges[self.right_min_dist_index]
        f_scan_msg.ranges[self.right_min_dist_index] = self.right_min_value


        # We calculate the angle in radians between both detections
        angle_left = f_scan_msg.angle_increment * self.left_min_dist_index
        angle_right = f_scan_msg.angle_increment * self.right_min_dist_index
        self.angle_between_left_right = angle_right - angle_left
        

        self.check_if_table()
        self.detection_table_side()
        
        self.scan_pub_.publish(f_scan_msg)
        

    def detection_table_side(self):
        S = self.laser_length
        if self.left_min_dist_index <= (3.0/16.0)*S and self.right_min_dist_index >= (13.0/16.0)*S:
            # Table legs on the back of the robot
            self.table_pos = "back"
        else:
            # Table Forwards or just in the middle of the legs
            self.table_pos = "front"

    
    def check_if_table(self, error = 0.05):
        """
        Based on cosine law, we get the distance between the two points
        based on the distance readings of each point an dthe angle
        If the value is around what the table should mesure, we say is a table
        """
        d1 = self.right_min_value
        d2 = self.left_min_value

        beta = self.angle_between_left_right
        aux = np.square(d2) + np.square(d1) - 2*d1*d2*np.cos(beta)
        w = np.sqrt(aux)

        
        width_ok = w >= self.Table_Width - error and w <= self.Table_Width + error
        if width_ok:
            distance_ok = d1 <= self.MAX_DIST_FROM_TABLE and d2 <= self.MAX_DIST_FROM_TABLE
            if not distance_ok:
                self.get_logger().debug("TABLE TOO FAR AWAY=d1="+str(d1)+",d2="+str(d2))
                self.is_table = False
            else:
                # We nee dto check that its INFRONT, not on the back
                table_is_front = self.table_pos == "front"
                if not table_is_front:
                    self.get_logger().debug("NOT IN FRONT="+str(self.table_pos))
                else:
                    self.get_logger().debug("IS TABLE IN FRONT =d1="+str(d1)+",d2="+str(d2)+"w="+str(w))
                    self.is_table = True
        else:
            self.get_logger().debug("WIDTH TABLE WRONG=w="+str(w))
            self.is_table = False
        

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

    def go_under_table(self,goal_handle):

        feedback_msg = GoUnderTable.Feedback()        

        # We do the Three fases: FIND, GO UNDER and CENTER
        feedback_msg.phase = "searching_for_table"
        goal_handle.publish_feedback(feedback_msg)
        self.find_table()

        feedback_msg.phase = "moving_under_table"
        goal_handle.publish_feedback(feedback_msg)
        self.get_under_table()

        feedback_msg.phase = "centering_under_table"
        goal_handle.publish_feedback(feedback_msg)
        self.center()

        # TODO: Debug to make it work
        # if "barista" in self.robot_name:
        #     feedback_msg.phase = "centering_barista_under_table"
        #     goal_handle.publish_feedback(feedback_msg)
        #     self.center_barista()
        
        feedback_msg.phase = "finished_go_under_table"
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info("Robot Found Table")
        goal_handle.succeed()
        result = GoUnderTable.Result()
        result.complete = True
        return result 

    def find_table(self):
        """
        Start rotating looking for detecting objects in boh sides at a maximum distance
        When done, it check if its a possible table
        If not then continues
        """
        
        rate = self.create_rate(10)

        while not self.is_table:
            
            self.rotate(direction="left", w=0.5, x=0.0)
            rate.sleep()

        self.stop()
        self.get_logger().info("Robot Found Table")

        return True 
    



    def get_under_table(self):
        """
        Gets Under the table until the front legs are 
        behind the robot, but close than the other legs
        """

        rate = self.create_rate(10)

        while self.table_pos != "back":

            # we want to make the angle of right and left min the closest as possible
            delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_right = self.right_min_dist_index - int(self.laser_length / 2)

            control_variable = -1*(delta_right - delta_left) / 100.0
            vel_w = self.angular_controller.update(control_variable)

            self.rotate(direction=None, w=vel_w, x=0.05)
            rate.sleep()

        self.stop()
        self.get_logger().info("Robot Under Table")

        return True 
        

    def center(self):
        """
        Mves forwards until boh four legs are at the same distance
        """
        # We Start from detectin on the back

        rate = self.create_rate(10)
        self.went_front_num = 0
        while self.table_pos == "back" or self.went_front_num <= 10:

            # we want to make the angle of right and left min the closest as possible
            delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_right = self.right_min_dist_index - int(self.laser_length / 2)

            control_variable = -1*(delta_right - delta_left) / 100.0
            vel_w = self.angular_controller.update(control_variable)

            self.rotate(direction=None, w=vel_w, x=0.02)
            rate.sleep()
            self.get_logger().warn("went_front_num="+str(self.went_front_num))
            if self.table_pos == "front":
                self.went_front_num += 1

        self.stop()
        self.get_logger().info("Robot is Centered under the Table")

        return True

    def center_barista(self):
        """
        Mves forwards until the legs in fornt of him
        are at a distance of self.MIN_DIST_FROM_TABLE
        """
        # We Start from detectin on the back

        rate = self.create_rate(10)

        d1 = self.right_min_value
        d2 = self.left_min_value
        close_enough = d1 <= self.MIN_DIST_FROM_TABLE and d2 <= self.MIN_DIST_FROM_TABLE
        far_enough = d1 >= self.MIN_DIST_FROM_TABLE+0.1 and d2 <= self.MIN_DIST_FROM_TABLE+0.1

        self.get_logger().warn("[d1,d2]=["+str(d1)+","+str(d2)+"],"+str(self.table_pos)+",c="+str(close_enough)+",f="+str(far_enough))
        while not close_enough:
            d1 = self.right_min_value
            d2 = self.left_min_value
            close_enough = d1 <= self.MIN_DIST_FROM_TABLE and d2 <= self.MIN_DIST_FROM_TABLE
            self.get_logger().warn("[d1,d2]=["+str(d1)+","+str(d2)+"],"+str(self.table_pos)+","+str(close_enough))
            # we want to make the angle of right and left min the closest as possible
            delta_left = int(self.laser_length / 2) - self.left_min_dist_index
            delta_right = self.right_min_dist_index - int(self.laser_length / 2)

            control_variable = -1*(delta_right - delta_left) / 100.0
            vel_w = self.angular_controller.update(control_variable)

            self.rotate(direction=None, w=vel_w, x=0.02)
            rate.sleep()

        self.stop()
        self.get_logger().warn("Robot Type Barista is Centered under the Table")

        return True

def main(args=None):
    rclpy.init(args=args)

    try:
        barista_aligner = Aligner() 
        executor = MultiThreadedExecutor(num_threads=4)
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
