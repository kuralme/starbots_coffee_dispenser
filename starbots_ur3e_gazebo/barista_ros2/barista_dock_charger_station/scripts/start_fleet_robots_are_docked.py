#! /usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from sensor_msgs.msg import BatteryState
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy

from barista_dock_charger_station.msg import FleetDockStatus

class MultiDockStatus(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('dock_charger_station_node')


        self.declare_parameter('fleet_name', "barista")
        self.declare_parameter('fleet_robots_number', 3)
        self.getting_params()

        self.fleet_docked_status_array = [False] * self.fleet_robots_number
        # create the service server object
        for i in range(self.fleet_robots_number):
            robot_name = self.fleet_name +"_"+str(i+1)
            docked_status_topic_name = robot_name+"/docked_status"
            
            qos_profile_subscriber = QoSProfile(depth=1)
            qos_profile_subscriber.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

            # Your own common callback
            self.get_logger().error("INIT Subscriber =="+str(docked_status_topic_name))
            self.create_subscription(   String, 
                                        docked_status_topic_name,
                                        self.dock_state_clb,
                                        qos_profile=qos_profile_subscriber)
            self.get_logger().error("INIT Subscriber =="+str(docked_status_topic_name)+"....DONE")

        
        # We cerate the docked status publisher of the fleet
        self.docked_fleet_status_topic_name = self.fleet_name + "/docked_status"
        qos_profile_publisher = QoSProfile(depth=1)
        # We make it QoSDurabilityPolicy.TRANSIENT_LOCAL so that it stays there for ever until changed
        qos_profile_publisher.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.docked_publisher_ = self.create_publisher(FleetDockStatus, self.docked_fleet_status_topic_name, qos_profile=qos_profile_publisher)
        # We initialise it to false
        self.fleet_dock_msgs = FleetDockStatus()
        self.fleet_dock_msgs.fleet_dock_status_array = [False] * self.fleet_robots_number
        self.fleet_dock_msgs.fleet_names_array = []
        for i in range(self.fleet_robots_number):
            robot_name = self.fleet_name +"_"+str(i+1)
            self.fleet_dock_msgs.fleet_names_array.append(robot_name)

        self.docked_publisher_.publish(self.fleet_dock_msgs)

    def dock_state_clb(self, msg):

        robot_name = msg.data.split("-")[0]
        robot_fleet_array_index = int(robot_name.split("_")[1]) - 1
        robot_dock_status = msg.data.split("-")[1]

        self.fleet_dock_msgs.fleet_dock_status_array[robot_fleet_array_index] = (robot_dock_status == "DOCKED")

        self.get_logger().error("New Message in_robot_name=="+str(msg.data)+", Republish ARRAY="+str(self.fleet_dock_msgs))

        self.docked_publisher_.publish(self.fleet_dock_msgs)


    def getting_params(self):

        self.fleet_name = self.get_parameter(
            'fleet_name').get_parameter_value().string_value
        

        self.fleet_robots_number = self.get_parameter(
            'fleet_robots_number').get_parameter_value().integer_value


        
        self.get_logger().info("## Dock Charger Manager for fleet_name ==="+str(self.fleet_name))
        self.get_logger().info("## Dock Charger Manager fleet_robots_number ==="+str(self.fleet_robots_number))

    


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor  
    mdt_obj = MultiDockStatus()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(mdt_obj)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()