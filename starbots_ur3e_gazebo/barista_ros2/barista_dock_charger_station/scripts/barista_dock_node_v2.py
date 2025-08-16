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

    def __init__(self, fov, robot_name, a_P, a_D, l_P, l_D):
        super().__init__('{}_dock_detector'.format(robot_name))
        
        # define variables
        self.fov = fov
        self.ranges = []
        self.dock_width = 0.30 # dock width in meters
        self.dock_width_th = 0.03 # detected dock width tolerance in meters
        self.max_dock_dist = 1.6 # maximum dock detected distance
        self.lidar_to_center = 0.08
        self.orientation_type = None
        self.lidar_readings_length = None
        self.obstacle = []
        self.mean_dock_ranges = None
        self.angular_controller = Controller(P=a_P, D=a_D) # setpoint 90 degrees
        self.linear_controller = Controller(P=l_P, D=l_D)
        
        # Converts the index number to corresponding angle
        self.index2angle = lambda r: r/(self.lidar_readings_length/self.fov)
        self.index2angle_rad = lambda range_list: [math.radians(self.index2angle(r)) for r in range_list]
        
        
        # declare parameters
        self.robot_name = robot_name
        self.declare_parameter('robot_name', robot_name)
        print(self.robot_name)
        
        # create callback group so they don't get blocked
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
            '{}/scan'.format(robot_name),
            self.scan_callback,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
            ),
            callback_group=self.group1
        )
        self.laser_subscription  # prevent unused variable warning
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            '{}/odom'.format(robot_name),
            self.odom_callback,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.BEST_EFFORT
            ),
            callback_group=self.group1
        )
        self.odom_subscription  # prevent unused variable warning

        
        # velocity publisher
        self.publisher = self.create_publisher(Twist, '{}/cmd_vel'.format(robot_name), 10)
        

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
            angle_th = 2 # angle tolerance (in degrees)
            front_lidar_angle = self.fov / 2
            docked = False
            centered = False
            robot_oriented = False
            self.mean_dock_ranges = None
            self.obstacle = []
            while not robot_oriented:
                self.process_scan()
                if len(self.obstacle):
                    # print("1: dock center angle: ", self.dock_center_angle)
                    if self.dock_center_angle > (front_lidar_angle + angle_th):
                        self.rotate("left", w=0.2)
                    elif self.dock_center_angle < (front_lidar_angle - angle_th):
                        self.rotate("left", w=0.2)
                    else:
                        self.stop_robot()
                        robot_oriented = True
                        centered = (abs(abs(self.dock_center_angle - self.dock_left_edge_angle) - abs(self.dock_center_angle - self.dock_right_edge_angle)) <= 10) & (self.alpha < 90) & (self.beta < 90)
                        # print("oriented, centered: {}".format(centered))
                   
                    
                else:
                    self.rotate("left", w=0.2)
                    print("dock NOT found, rotating to try and find it...")
                    
            if not centered:
                parallel = False
                direction = None
                while not parallel:
                    self.process_scan()
                    if len(self.obstacle):
                        theta = front_lidar_angle - self.dock_left_edge_angle 
                        if direction is None:
                            if theta + 1 < self.beta:
                                # print("ROTATIIIIING LEFT")
                                print(theta, self.beta)
                                direction = "left"
                            elif theta - 1  > self.beta:
                                # print("ROTATIIIIIIING RIGHT")
                                print(theta, self.beta)
                                direction = "right"

                        print(theta, self.beta)
                        if (theta + 1 < self.beta) or (theta - 1  > self.beta):
                            self.rotate(direction, w=0.10)
                            # print("ROTATIIIIIIING   " + direction)
                        else:
                            self.stop_robot()
                            # print("ROBOT IS NOW PARALELL")
                            parallel = True
                        
                    else:
                        print("no obstacle detected while paralleling")
                        direction = None
                
                go_forward = self.beta < self.alpha
                print(self.beta, self.alpha, go_forward)
                aligned = False
                while not aligned:
                    self.process_scan()
                    
                    if len(self.obstacle):
                        print("omega: {}".format(math.degrees(self.omega)))
                        if (abs(math.degrees(self.omega) - 90) >= 2):
                            # print("centering to dock...")
                            if go_forward:
                                self.forward()
                            else:
                                self.backward()
                        else:
                            odom_initial = self.odom_pose.pose.position.x
                            while abs(self.odom_pose.pose.position.x - odom_initial) < self.lidar_to_center:
                                self.forward()
                            self.stop_robot()
                            aligned = True
                    else:
                        print("no obstacle, not doing last part")
                
                while not centered:
                    self.process_scan()
                    # print("2: dock center angle: ", self.dock_center_angle)
                    centered = self.dock_center_angle > 179
                    if not centered:
                        self.rotate("right", w=0.2)
                    elif self.dock_center_angle > 181:
                        self.rotate("left", w=0.2)
                    else:
                        self.stop_robot()
                        # print("CENTERED")
                
            else:
                print("CENTERED")
                
            
            # print("LAST PART")
            while not docked:
                # self.process_scan()
                
                perform_control = True
                # Set linear controller setpoint
                initial_odom = self.odom_pose.pose.position.x
                goal_odom = None
                self.linear_controller.setPoint(goal_odom)
                # Set angular controller setpoint
                th_setpoint = 2
                setpoint = front_lidar_angle
                self.angular_controller.setPoint(math.radians(setpoint))
                
                # while (self.front_range > 0.16):
                while perform_control:
                    self.process_scan()
                    control_variable = math.radians(self.dock_center_angle)
                    # CONTROLLER APPROACH
                    w = self.angular_controller.update(control_variable) # Get angular velocity
                    # print(math.degrees(control_variable), -w)
                    perform_control = (self.front_range > 0.2) or (math.degrees(control_variable) > setpoint+th_setpoint) or (math.degrees(control_variable) < setpoint - th_setpoint)
                    # Move according to the controller
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
        
    def process_scan(self): 
        np_ranges = np.array(self.ranges)          
        abs_np_reading_jumps = np.abs(np.diff(np_ranges))   # absolute values of the reading jumps of each consecutive value
        condition = abs_np_reading_jumps > 0.17             # if reading jump is more than 17 cm, it passes the condition
        self.big_reading_jumps = np.where(condition)[0]          # extracts index of reading jumps that pass the condition 
        
        self.continuous_readings = [np_ranges[:self.big_reading_jumps[0]]] # creates array of first readings section until first big reading jump   
        for current, next in zip(self.big_reading_jumps, self.big_reading_jumps[1:]): # loops through all big reading jumps indexes
            self.continuous_readings.append(np_ranges[1:][current:next])    # appends consecutive readings, np_ranges[1:] means use every reading but the 0th, due to the nature of the diff() function
        self.continuous_readings.append(np_ranges[1:][self.big_reading_jumps[-1]:]) # appends the last readings section from last big reading
                                                                               # jump to end of array
        # self.obstacle = [] # creates empty obstacle array
        orientation_type = None
        for i, reading in enumerate(self.continuous_readings):  # loops through each continuous section AND each reading in that section
                                                                # in continuous_readings array
            eval_reading = []                                   # creates empty reading array
            
            if i == 0:                   # if only part of the dock corresponds to the first continuous section
                orientation_type = 0
                # print("extension commenced")
                pre_angle = self.big_reading_jumps[0] + (self.lidar_readings_length - self.big_reading_jumps[-1]) # adds the indexes of the first continuous section and the last to obtain angle between the distance of the dock edges and the robot
                eval_reading = reading.tolist()                                 # converts array to list
                if self.fov == 360:
                    eval_reading.extend(np_ranges[1:][self.big_reading_jumps[-1]:])      # extends the first continuous section with the last
            elif i == len(self.continuous_readings)-1:                          # if only part of the dock corresponds to the last continuous section
                orientation_type = 1
                # print("last extension")
                pre_angle = self.lidar_readings_length - self.big_reading_jumps[-1]  # indexes between the last big jump and the end of array to obtain angle
                if self.fov == 360:
                    eval_reading = np_ranges[:self.big_reading_jumps[0]].tolist()        # get first continuous section until first big jump in a list
                    eval_reading.extend(reading.tolist())                           # extend last continuous section with the first
                else:
                    eval_reading = reading.tolist()
            else:                          # the entire dock is somewhere in between of the array
                orientation_type = 2
                # print("normal execution") 
                eval_reading = reading # no need to manipulate the continuous section
                pre_angle = self.big_reading_jumps[i] - self.big_reading_jumps[i - 1] # difference of indexes between big jumps to obtain angle
 
            
            try:
                self.b = eval_reading[0]             # first value of the current continuous section
                self.a = eval_reading[-1]            # last value of the current continuous section. They make a triangle with robot
                gamma = self.index2angle(pre_angle)  # angle between a and b
                c = math.sqrt(math.pow(self.a,2) + math.pow(self.b,2) - 2 * self.a * self.b * math.cos(math.radians(gamma))) # cosine law to find continuous section length
                if c > (self.dock_width-self.dock_width_th) and c < (self.dock_width+self.dock_width_th):       # if c is similar to the length of the dock
                    if self.b < self.max_dock_dist:                 # if the first value of the current continuous section is closer than 2m
                        if self.mean_dock_ranges is None:
                            self.mean_dock_ranges = np.nanmean(eval_reading)
                        if np.abs(np.nanmean(eval_reading) - self.mean_dock_ranges) < 0.1:
                              
                            self.obstacle = eval_reading.copy() # save continous section as obstacle (dock)
                            self.mean_dock_ranges = np.nanmean(eval_reading)
                            self.orientation_type = orientation_type
                            
                                
                            dock_right_edge_ix = self.big_reading_jumps[i-1]
                            dock_left_edge_ix = self.big_reading_jumps[i]
                            # dock_center_ix = dock_right_edge_ix + (dock_left_edge_ix - dock_right_edge_ix)/2
                            dock_center_ix = np.where(self.obstacle == np.nanmin(self.obstacle))[0][0] + dock_right_edge_ix
                            self.dock_right_edge_angle = self.index2angle(dock_right_edge_ix)
                            self.dock_left_edge_angle = self.index2angle(dock_left_edge_ix)
                            self.dock_center_angle = self.index2angle(dock_center_ix)
                            # print(self.dock_center_angle)
                            self.alpha = math.degrees( math.acos((math.pow(self.b,2) + math.pow(c,2) - math.pow(self.a,2)) / (2*self.b*c)) )
                            # self.alpha = math.degrees( math.asin((self.a/c) * math.sin(math.radians(gamma))))  # law of sines
                            self.beta = 180 - self.alpha - gamma
                            
                            # obtain omega
                            d = math.sqrt( math.pow(self.a,2) + math.pow(self.dock_width/2,2) - self.a*self.dock_width*math.cos(math.radians(self.beta)) )
                            self.omega = math.asin( (self.a/d) * math.sin(math.radians(self.beta)) )
                            print(len(self.obstacle), self.dock_center_angle)
                            break
                        else:
                            # print("there is a different obstacle in the range:  ", np.nanmean(eval_reading), self.big_reading_jumps[i-1])
                            pass
                    else:
                
                           
                        pass
            except IndexError:
                pass
            
       

def main(args=None, parser_args=None):
    rclpy.init(args=args)
    
    try:
        dock_detector = DockDetector(parser_args.fov, parser_args.robot_name, parser_args.a_P, parser_args.a_D, parser_args.l_P, parser_args.l_D)   # class object
        
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
    parser = argparse.ArgumentParser(description='Dock node')
    parser.add_argument('-robot_name', help='Robot name', default='barista_1')
    parser.add_argument('-fov', type=float, help='Laser field of vision', default=360)
    parser.add_argument('-a_P', type=float, help='angular controller proportional term', default=0.9)
    parser.add_argument('-a_D', type=float, help='angular controller derivative term', default=0.05)
    parser.add_argument('-l_P', type=float, help='linear controller proportional term', default=0.9)
    parser.add_argument('-l_D', type=float, help='linear controller derivative term', default=0.05)
    args = parser.parse_args()
    main(args=None, parser_args=args)


