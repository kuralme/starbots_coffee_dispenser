#! /usr/bin/env python3

import math
import rclpy
import argparse
import numpy as np
from cmath import asin
from turtle import forward
from rclpy.node import Node
from statistics import mean
from time import sleep, time
from std_msgs.msg import String
from math import degrees, radians
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import TwistWithCovariance
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, BatteryState
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rclpy.callback_groups import ReentrantCallbackGroup
from barista_dock_charger_station.msg import FleetDockStatus
from barista_dock_charger_station.srv import DockChargerStation
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


class DockDetector(Node):

    def __init__(self):
        
        

        super().__init__('dock_detector')
        
        # declare parameters
        self.declare_parameter('robot_name', "barista_X")
        self.getting_params()

        # define variables
        self.ranges = []
        self.max_dock_dist = 1.6 # maximum dock detected distance
        self.lidar_to_center = 0.08
        self.orientation_type = None
        self.lidar_readings_length = None
        self.obstacle = []
        self.mean_dock_ranges = None
        self.a_P = 0.9
        self.a_D = 0.05
        self.angular_controller = Controller(P=self.a_P, D=self.a_D) # setpoint 90 degrees
        
        # Converts the index number to corresponding angle
        self.index2angle = lambda r: r/(self.lidar_readings_length/self.fov)
        self.index2angle_rad = lambda range_list: [math.radians(self.index2angle(r)) for r in range_list]
        
        
        self.group1 = ReentrantCallbackGroup()
        self.group2 = ReentrantCallbackGroup()
        
        # service server object
        self.dock_charger_server_name = self.robot_name + "/dock_charger_station"
        self._service_server = self.create_service(DockChargerStation, 
                                                   self.dock_charger_server_name, 
                                                   self.server_callback, 
                                                   callback_group=self.group1)
        
        # lidar subscriber, with callback part of group1
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '{}/scan'.format(self.robot_name),
            self.scan_callback,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
            ),
            callback_group=self.group1
        )
        self.laser_subscription  # prevent unused variable warning
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '{}/odom'.format(self.robot_name),
            self.odom_callback,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
            ),
            callback_group=self.group1
        )
        self.odom_subscription  # prevent unused variable warning

        
        # velocity publisher
        self.publisher = self.create_publisher(Twist, '{}/cmd_vel'.format(self.robot_name), 10)
        

        # velocity command
        self.cmd = Twist()

        
        # We start the publisher to talk with the battery simulated node
        # The dock can be in two modes: Someone is docked, or noeone is docked
        # If they are docked, they can be: charging or full
        # Options: DOCKED_CHARGING, DOCKED_FULL, DISCONECTED
        self.dock_state = "DISCONECTED"

        self.update_battery_charge_state_topic_name = self.robot_name + \
            "/update_battery_charge_state"
        self.update_battery_charge_state_publisher_ = self.create_publisher(
            String, self.update_battery_charge_state_topic_name, 1)

        # Now we start the subscriber of the battery system top monitor it
        self.battery_state_topic_name = self.robot_name + "/battery_state"

        self.subscriber = self.create_subscription(
            BatteryState,
            self.battery_state_topic_name,
            self.battery_state_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
            callback_group=self.group1
        )

        # We create the docked status publisher
        self.docked_status_topic_name = self.robot_name + "/docked_status"
        qos_profile_publisher = QoSProfile(depth=1)
        # We make it QoSDurabilityPolicy.TRANSIENT_LOCAL so that it stays there for ever until changed
        qos_profile_publisher.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.docked_publisher_ = self.create_publisher(
            String, self.docked_status_topic_name, qos_profile=qos_profile_publisher)
        # We initialise it to false
        dock_msgs = String()
        dock_msgs.data = self.robot_name+"-"+"UNDOCKED"
        self.docked_publisher_.publish(dock_msgs)

    def getting_params(self):

        self.robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value

        
    def battery_state_callback(self, msg):
        """
        # Power supply status constants
        uint8 POWER_SUPPLY_STATUS_UNKNOWN = 0
        uint8 POWER_SUPPLY_STATUS_CHARGING = 1
        uint8 POWER_SUPPLY_STATUS_DISCHARGING = 2
        uint8 POWER_SUPPLY_STATUS_NOT_CHARGING = 3
        uint8 POWER_SUPPLY_STATUS_FULL = 4

        """
        if self.dock_state != "DISCONECTED":
            self.percentage = msg.percentage
            self.power_supply_status = msg.power_supply_status
            self.get_logger().info("Monitoring Battery percentage of Robot " +
                                   self.robot_name+"==>"+str(self.percentage))

            if self.power_supply_status == 1:
                self.dock_state = "DOCKED_CHARGING"
            elif self.power_supply_status == 4:
                self.dock_state = "DOCKED_FULL"

            self.get_logger().info("Charging Station Robot = " +
                                   self.robot_name+" state="+str(self.dock_state))

        else:
            pass

    def set_battery_state(self, new_state):
        """
        Allowed states are:
            "DISCHARGING"
            "CHARGING"
            "FAST_CHARGING"
            "FAST_DISCHARGING"
            "DISCHARGING"
            "LOW_BATTERY_SET"

        """
        allowed_states_array = ["DISCHARGING",
                                "CHARGING",
                                "FAST_CHARGING",
                                "FAST_DISCHARGING",
                                "DISCHARGING",
                                "LOW_BATTERY_SET"]

        if new_state not in allowed_states_array:
            self.get_logger().error("New batter state NOt supported="+str(new_state))
        else:
            bat_state_msg = String()
            bat_state_msg.data = new_state

            self.update_battery_charge_state_publisher_.publish(bat_state_msg)

    def server_callback(self, request, response):
        if request.dock_mode:
            docked = False            
            while not docked:                
                perform_control = True
                # Set angular controller setpoint
                setpoint = 0
                self.angular_controller.setPoint(setpoint)
                
                while perform_control:
                    control_variable = self.right_dock - self.left_dock
                    # CONTROLLER APPROACH
                    w = self.angular_controller.update(control_variable) # Get angular velocity
                    perform_control = (self.front_range > 0.2) 
                    self.rotate(direction=None, w=-w, x=0.1)
                    
                self.stop_robot()
                dock_msgs = String()
                dock_msgs.data = self.robot_name+"-"+"DOCKED"
                self.docked_publisher_.publish(dock_msgs)
                self.set_battery_state(new_state="FAST_CHARGING")
                self.dock_state = "DOCKED_CHARGING"
                docked = True
            print("DOCKED!")
                
                
                
                
            response.success = True
            return response   
        else:
            self.backward()
            sleep(6)
            self.stop_robot()
            response.success = True
            dock_msgs = String()
            dock_msgs.data = self.robot_name+"-"+"UNDOCKED"
            self.docked_publisher_.publish(dock_msgs)
            self.set_battery_state(new_state="DISCHARGING")
            self.dock_state = "DISCONECTED"
            print("UNDOCKED!")
            return response
             
        
    def scan_callback(self, msg):
        # length of ranges is 720 for simulation, 1000 for rplidar a1, 726 for Hokuyo URG-04LX-UG01 and
        # front of robot varies from lidar to lidar
        self.ranges = msg.ranges                            # store data in ranges class variable
        self.lidar_readings_length = len(self.ranges)       # number of lidar readings
        self.front_range_ix = int(self.lidar_readings_length/2)
        self.front_range = self.ranges[self.front_range_ix]
        self.right_side_ix = int(self.lidar_readings_length/4)
        self.right_side = self.ranges[self.right_side_ix]
        self.left_dock_ix = self.front_range_ix + 14
        self.left_dock = self.ranges[self.left_dock_ix]
        self.right_dock_ix = self.front_range_ix - 14
        self.right_dock = self.ranges[self.right_dock_ix]
        # self.process_scan()
        
    def odom_callback(self, msg):
        self.odom_pose = msg.pose
        self.odom_twist = msg.twist
        
    def rotate(self, direction, w=0.5, x=0.0):
        if direction == "left":
            self.cmd.angular.z = w
            self.cmd.linear.x = x
        elif direction == "right":
            self.cmd.angular.z = -w
            self.cmd.linear.x = x
        elif direction is None:
            self.cmd.angular.z = w
            self.cmd.linear.x = x
        self.publisher.publish(self.cmd)
        
    def stop_robot(self):
        self.cmd.angular.z = 0.0
        self.cmd.linear.x = 0.0
        self.publisher.publish(self.cmd)
        
    def forward(self):
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)
        
    def backward(self):
        self.cmd.linear.x = -0.05
        self.cmd.angular.z = 0.0
        self.publisher.publish(self.cmd)
        
       

def main(args=None):
    rclpy.init(args=args)
    
    try:
        dock_detector = DockDetector()   # class object
        
        executor = MultiThreadedExecutor(num_threads=3)     # create multi-threaded executor with number of callback functions
        executor.add_node(dock_detector)                    # add the class object to the executor
        
        try:
            executor.spin()
        finally:
            executor.shutdown()
            dock_detector.destroy_node()
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    
    main()


