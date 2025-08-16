#! /usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, DurabilityPolicy, QoSProfile

from barista_dock_charger_station.srv import DockChargerStation
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy


class Service(Node):

    def __init__(self):
        # Here we have the class constructor

        # call the class constructor to initialize the node as service_stop
        super().__init__('dock_charger_station_node')

        self.declare_parameter('robot_name', "barista_GENERIC")
        self.declare_parameter('max_dock_distance_param', 0.5)
        self.declare_parameter('undock_distance_param', 0.3)
        self.declare_parameter('dock_distance_param', 0.3)
        self.getting_params()

        # create the service server object
        self.dock_charging_station_server_name = self.robot_name + "/dock_charger_station"
        self.srv = self.create_service(DockChargerStation,
                                       self.dock_charging_station_server_name,
                                       self.docker_charger_station_clb)

        # Start subscriber for the models in gazebo state info
        self.model_states_topic_name = '/gazebo/model_states'
        self.model_states = ModelStates()
        self.subscriber = self.create_subscription(
            ModelStates,
            self.model_states_topic_name,
            self.model_states_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))

        # We cerate a service client for teleporting any gazebo model
        self.set_entity_srv_name = '/gazebo/set_entity_state'
        self.client = self.create_client(
            SetEntityState, self.set_entity_srv_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service "+self.set_entity_srv_name +
                                   "not available, waiting again...")

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
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE))

        # We create the docked status publisher
        self.docked_status_topic_name = self.robot_name + "/docked_status"
        qos_profile_publisher = QoSProfile(depth=1)
        # We make it QoSDurabilityPolicy.TRANSIENT_LOCAL so that it stays there for ever until changed
        qos_profile_publisher.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

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

    def getting_params(self):

        self.robot_name = self.get_parameter(
            'robot_name').get_parameter_value().string_value

        self.max_dock_distance_param = self.get_parameter(
            'max_dock_distance_param').get_parameter_value().double_value

        self.undock_distance_param = self.get_parameter(
            'undock_distance_param').get_parameter_value().double_value

        self.dock_distance_param = self.get_parameter(
            'dock_distance_param').get_parameter_value().double_value

        self.get_logger().info("## Dock Charger Manager for robot_name ==="+str(self.robot_name))
        self.get_logger().info("## Dock Charger Manager max_dock_distance_param ===" +
                               str(self.max_dock_distance_param))
        self.get_logger().info("## Dock Charger Manager undock_distance_param ===" +
                               str(self.undock_distance_param))
        self.get_logger().info("## Dock Charger Manager dock_distance_param ===" +
                               str(self.dock_distance_param))

    def model_states_callback(self, msg):
        self.model_states = msg
        self.get_logger().debug('I receive: "%s"' % str(self.model_states))

    def calculate_distance_between_two_poses(self, pose_1, pose_2):
        p1 = pose_1.position
        p2 = pose_2.position

        delta2_x = math.pow(p2.x - p1.x, 2)
        delta2_y = math.pow(p2.y - p1.y, 2)

        distance = math.sqrt(delta2_x + delta2_y)
        return distance

    def get_closest_charging_station_to_robot(self, robot_name, distance_limit):
        """
        It searches for the closest charging station pose that is withing the range of the infrared
        If charging station found, the result_info=charging_station_X
        """
        robot_result, robot_pose = self.get_pose_model(model_name=robot_name)
        if robot_result:
            # We search for all the charging stations and their poses
            charging_station_dict = self.get_all_charging_station_data()
            min_dist = distance_limit
            min_dist_pose = Pose()
            found_min = False
            result_info = "No charger found within the distance of " + \
                str(distance_limit)
            for charging_station_name, charging_station_pose in charging_station_dict.items():
                distance = self.calculate_distance_between_two_poses(
                    robot_pose, charging_station_pose)
                self.get_logger().info("Distance to "+str(charging_station_name)+" == "+str(distance))
                if distance < min_dist:
                    self.get_logger().info("Its a minimum...")
                    min_dist = distance
                    min_dist_pose = charging_station_pose
                    found_min = True
                    result_info = charging_station_name

            pose = min_dist_pose
            result = found_min

        else:
            result_info = "Robot "+str(robot_name)+" not found"
            result = False
            pose = Pose()

        return result_info, result, pose

    def get_all_charging_station_data(self):
        """
        We get all the models poses  that have as name `charging_station_X`
        """
        names_array = self.model_states.name
        charging_station_dict = {}
        for i in range(len(names_array)):
            if "charging_station_" in names_array[i]:
                pose = self.model_states.pose[i]
                charging_station_dict[names_array[i]] = pose

        return charging_station_dict

    def get_pose_model(self, model_name):

        names_array = self.model_states.name

        index = -1

        if model_name in names_array:
            index = names_array.index(model_name)

        success = False
        pose = Pose()
        if index != -1:
            pose = self.model_states.pose[index]
            self.get_logger().info("Model "+model_name+" pose found ="+str(pose))
            success = True
        else:
            pass

        return success, pose

    def teleport_robot(self, robot_name, pose, teleport_frame="world"):
        """
        It teleports the given gazebo model to the given pose
        rosservice call /gazebo/set_model_state '{model_state: { model_name: MODEL+NAME, pose: { position: { x: 0, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
        ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState 
        """

        request_obj = SetEntityState.Request()
        request_obj.state.name = robot_name
        request_obj.state.pose = pose

        request_obj.state.reference_frame = teleport_frame

        self.future = self.client.call_async(request_obj)

        response_obj = SetEntityState.Response()

        if self.future.done():
            try:
                response_obj = self.future.result()
            except Exception as e:
                # Display the message on the console
                self.get_logger().error(
                    "Service "+self.set_entity_srv_name+"call failed ==>"+str(e))
            else:
                # Display the message on the console
                self.get_logger().error(
                    'the robot was teleported to ==>'+str(pose))

        self.get_logger().error("Finished teleport_robot...")

        return response_obj.success

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

    def dock_nearest_station(self, robot_name, distance_limit=0.5, dock_distance=0.0):

        # We look for the closest station
        result_info, get_pose_result, dock_pose = self.get_closest_charging_station_to_robot(
            robot_name, distance_limit=distance_limit)

        if get_pose_result:
            self.get_logger().error("Teleporting...")
            # TODO: HARDCODED DISTANCE
            dock_pose = Pose()
            dock_pose.position.x += -1*dock_distance

            self.get_logger().error(">>>FRAME==="+str(result_info))

            result = self.teleport_robot(
                robot_name=robot_name, pose=dock_pose, teleport_frame=result_info)
            self.get_logger().error("Teleporting...DONE=="+str(result))
            # We change teh state to DOCKED_CHARGING
            # Options: DOCKED_CHARGING, DOCKED_FULL, DISCONECTED
            self.set_battery_state(new_state="FAST_CHARGING")
            self.dock_state = "DOCKED_CHARGING"
        else:
            self.get_logger().error(result_info)
            result = False

        return result

    def undock_current_station(self, robot_name, distance_limit=0.5, undock_rear_distance=0.35):

        # We look for the closest station
        result_info, get_pose_result, dock_pose = self.get_closest_charging_station_to_robot(
            robot_name, distance_limit=distance_limit)

        # We place it in front of it
        teleport_frame = result_info
        if get_pose_result:
            print("UNDOCK")
            dock_pose = Pose()
            dock_pose.position.x = -1.0*undock_rear_distance
            result = self.teleport_robot(
                robot_name=robot_name, pose=dock_pose, teleport_frame=teleport_frame)
            # Options: DOCKED_CHARGING, DOCKED_FULL, DISCONECTED
            self.set_battery_state(new_state="DISCHARGING")
            self.dock_state = "DISCONECTED"
            print("UNDOCK DONE")
        else:
            self.get_logger().error(result_info)
            result = False

        return result

    def docker_charger_station_clb(self, request, response):

        if request.dock_mode:

            self.get_logger().info('Starting Dummy Dock to Charging Station')
            response.success = self.dock_nearest_station(
                robot_name=self.robot_name,
                distance_limit=self.max_dock_distance_param,
                dock_distance=self.dock_distance_param)
            self.get_logger().info(
                'Starting Dummy Dock to Charging Station...DONE--RESPONSE='+str(response.success))

            dock_msgs = String()
            dock_msgs.data = self.robot_name+"-"+"DOCKED"
            self.docked_publisher_.publish(dock_msgs)

        else:
            self.get_logger().info('Starting Dummy UNdock to Charging Station')
            # We set the undock limit to the same as the self.dock_distance_param
            # Because if its docked, thatis the distance, we add a 10% just in case
            # The robot moved while there
            response.success = self.undock_current_station(
                robot_name=self.robot_name,
                distance_limit=self.dock_distance_param*1.1,
                undock_rear_distance=self.undock_distance_param)
            self.get_logger().info(
                'Starting Dummy UNdock to Charging Station...DONE--RESPONSE='+str(response.success))

            dock_msgs = String()
            dock_msgs.data = self.robot_name+"-"+"UNDOCKED"
            self.docked_publisher_.publish(dock_msgs)

        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
