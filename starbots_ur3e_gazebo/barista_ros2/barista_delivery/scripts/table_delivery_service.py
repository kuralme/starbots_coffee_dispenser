#! /usr/bin/env python3

import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile

from rmf_ingestor_msgs.msg import IngestorState
from rmf_ingestor_msgs.msg import IngestorResult
from rmf_ingestor_msgs.msg import IngestorRequest

from rmf_dispenser_msgs.msg import DispenserState
from rmf_dispenser_msgs.msg import DispenserResult
from rmf_dispenser_msgs.msg import DispenserRequest

from std_msgs.msg import String

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String

#Action client related stuff
from rclpy.action import ActionClient
from barista_table_finder.action import GoUnderTable

from barista_interfaces.srv import TextCommand

class TableDeliveryService(Node):

    def __init__(self, robot_name):

        self._robot_name = robot_name

        self.table_placer_dispenser_name="table_picker_service"
        self.table_picker_ingestor_name="table_placer_service"
        

        super().__init__('manual_dispenser')

        self.group1 = MutuallyExclusiveCallbackGroup()
        self.group2 = MutuallyExclusiveCallbackGroup()
        self.group3 = MutuallyExclusiveCallbackGroup()
        self.group_ac = MutuallyExclusiveCallbackGroup()
        self.group_pc = MutuallyExclusiveCallbackGroup()

        # TableDeliveryService STATE
        self.robot_state = "iddle"

        # Publish into the piston topic
        self.init_table_picker_dispenser()
        self.init_table_placer_ingestor()
        self.init_table_finder_action_client()

        self.timer_period = 0.2
        self.timer = self.create_timer(
            self.timer_period, self.timer_callback, callback_group=self.group3)

        self.get_logger().warning("TableDeliveryService READY for "+str(self.table_picker_ingestor_name)+","+str(self.table_placer_dispenser_name))

    def robot_is_available(self):
        """
        ReturnsTrue if the robot isn't doing anything
        """
        return self.robot_state == "iddle"


    def init_piston_service(self):
        self.piston_client = self.create_client(TextCommand, '/piston_control', callback_group=self.group_pc)
        # checks once per second if a Service matching the type and name of the Client is available.
        while not self.piston_client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service /piston_control not available, waiting again...')
        self.get_logger().info('service /piston_control READY')

        self.req = TextCommand.Request()

    def init_table_finder_action_client(self):
        self.table_finder_phase = "nothing"
        self.ac_name = "/"+self._robot_name+"/go_under_table"
        self._table_finder_ac = ActionClient(self, GoUnderTable, self.ac_name, callback_group=self.group_ac)


    def init_table_placer_ingestor(self):
        self._ingestor_state_pub = self.create_publisher(
            IngestorState, "/ingestor_states", 10)
        self._ingestor_result_pub = self.create_publisher(
            IngestorResult, "/ingestor_results", 10)

        self.current_ingestor_state = IngestorState()
        self.latest_ingestor_request = IngestorRequest()

        self.ingesting_now_flag = False
        self.processing_ingestor_request_flag = False

        self._request_sub = self.create_subscription(
            IngestorRequest,
            '/ingestor_requests',
            self.ingestor_request_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group1)

    def init_table_picker_dispenser(self):
        self._dispenser_state_pub = self.create_publisher(
            DispenserState, "/dispenser_states", 10)
        self._rdispenser_result_pub = self.create_publisher(
            DispenserResult, "/dispenser_results", 10)

        self.current_dispenser_state = DispenserState()
        self.latest_dispenser_request = DispenserRequest()

        self.dispensing_now_flag = False
        self.processing_dispenser_request_flag = False

        self._request_sub = self.create_subscription(
            DispenserRequest,
            '/dispenser_requests',
            self.dispenser_request_cb,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.group2)

    ##############
    # MAIN LOOP
    def timer_callback(self):

        if self.robot_is_available():
            # We aren't doing anything
            
            self.ingestor_state_publish(dispense=False)
            self.dispenser_state_publish(dispense=False)

            self.get_logger().info(self._robot_name+" is iddle")

        else:

            if "dispenser" in self.robot_state:
                if "table_search_finished" in self.robot_state:
                    # We have to call the piston service
                    self.robot_state = "dispenser-piston_going_up"
                    self.send_piston_request(direction="up")                    
                elif "piston_going_up" in self.robot_state:
                    self.dispenser_state_publish(dispense=True)
                    result = self.check_piston_state("up")
                    if "finished" in result:
                        self.robot_state = "iddle"
                        self.send_dispenser_response(DispenserResult.SUCCESS)
                    elif "error" in result:
                        self.robot_state = "iddle"
                        self.send_ingestor_response(DispenserResult.FAILED)
                    else:
                        self.dispenser_state_publish(dispense=True)
                else:
                    self.dispenser_state_publish(dispense=True)

            elif "ingestor" in self.robot_state:
                if "piston_going_down" in self.robot_state:
                    self.ingestor_state_publish(dispense=True)
                    result = self.check_piston_state("down")
                    if "finished" in result:
                        self.robot_state = "iddle"
                        self.send_ingestor_response(IngestorResult.SUCCESS)
                    elif "error" in result:
                        self.robot_state = "iddle"
                        self.send_ingestor_response(IngestorResult.FAILED)
                    else:
                        self.ingestor_state_publish(dispense=True)
                else:
                    self.ingestor_state_publish(dispense=True)
            else:
                self.get_logger().error(self._robot_name+" has ERROR in robot_state="+str(self.robot_state))
            
            
    ### PISTON STUF
    def send_piston_request(self, direction):
        if direction =="up":
            # send the request
            self.future = self.piston_client.call_async(self.req)
        elif direction == "down":
            # send the request
            self.future = self.piston_client.call_async(self.req)
        else:
            pass

    def check_piston_state(self, direction):
        client = self.piston_client                

        rclpy.spin_once(client)
        final_result = "piston_moving"
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running. 
                # If the Service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
                final_result = response.message
            except Exception as e:
                # Display the message on the console
                client.get_logger().info('Service call failed %r' % (e,))
                final_result = direction+"-error"
        else:
            pass

        return final_result

        

    ### INGESTOR STUFF
    def ingestor_state_publish(self, dispense):

        self.current_ingestor_state.time = self.get_clock().now().to_msg()

        if dispense:
            self.current_ingestor_state.mode = IngestorState.BUSY
            self.current_ingestor_state.request_guid_queue = {
                self.latest_ingestor_request.request_guid}
        else:
            self.current_ingestor_state.mode = IngestorState.IDLE
            self.current_ingestor_state.request_guid_queue.clear()

        self._ingestor_state_pub.publish(self.current_ingestor_state)

    def send_ingestor_response(self, status):

        response = IngestorResult()
        response.time = self.get_clock().now().to_msg()
        response.status = status
        response.request_guid = self.latest_ingestor_request.request_guid
        response.source_guid = self._manual_ingestor_name

        self._ingestor_result_pub.publish(response)

    def ingestor_request_cb(self, msg):

        if msg.target_guid == self._manual_ingestor_name:
            if self.robot_is_available():
                # To avoid that the callback publishes states
                self.robot_state = "ingestor-init"

                self.latest_ingestor_request = msg

                self.ingestor_state_publish(dispense=True)
                self.send_ingestor_response(IngestorResult.ACKNOWLEDGED)
                
                self.send_piston_request(direction="down") 
                self.robot_state = "ingestor-piston_going_down"
        else:
            self.get_logger().warning("This Ingestor request isn't for me =" +
                                      str(msg.target_guid)+" not ="+str(self._manual_ingestor_name))
    #################

    ##### DISPENSER STUFF
    def dispenser_request_cb(self, msg):

        if msg.target_guid == self.table_placer_dispenser_name:
            if self.robot_is_available():
                # To avoid that the callback publishes states
                self.robot_state = "dispenser-init"

                self.latest_dispenser_request = msg
                self.dispenser_state_publish(dispense=True)
                self.send_dispenser_response(DispenserResult.ACKNOWLEDGED)

                self.send_goal(search=True)

                self.robot_state = "dispenser-searching_for_table"
            else:
                self.get_logger().warning("The robot "+str(self._robot_name)+" is bussy with a task="+str(self.processing_dispenser_request_flag))
        else:
            self.get_logger().warning("This Dispenser request isn't for me =" +
                                      str(msg.target_guid)+" not ="+str(self.table_placer_dispenser_name))
    
    def dispenser_state_publish(self, dispense):

        if dispense:
            self.current_dispenser_state.mode = DispenserState.BUSY
            self.current_dispenser_state.request_guid_queue = {
                self.latest_dispenser_request.request_guid}
        else:
            self.current_dispenser_state.mode = DispenserState.IDLE
            self.current_dispenser_state.request_guid_queue.clear()

        self._dispenser_state_pub.publish(self.current_dispenser_state)

    
    def send_dispenser_response(self, status):

        response = DispenserResult()
        response.time = self.get_clock().now().to_msg()
        response.status = status
        response.request_guid = self.latest_dispenser_request.request_guid
        response.source_guid = self._manual_dispenser_name

        self._rdispenser_result_pub.publish(response)

    ################################################

    # AC stuff
    def send_goal(self, search=True):
        """
        If search=True, means that we wantteh robot to search for the table
        if False, then it will triguer the go out of the table
        """
        goal_msg = GoUnderTable.Goal()
        goal_msg.start = search

        self._table_finder_ac.wait_for_server()
        self._send_goal_future = self._table_finder_ac.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Table finder Result: {0}'.format(result.complete))
        self.robot_state = "dispenser-table_search_finished"

    def feedback_callback(self, feedback_msg):
        table_finder_phase = feedback_msg.phase
        self.robot_state = "dispenser-"+self.table_finder_phase
        self.get_logger().info(
            'Table Finder Feeback Recieved: {0}'.format(self.robot_state))
    ###################################################




def main(args=None):

    rclpy.init(args=args)

    simple_publisher = TableDeliveryService("barista_1")

    num_threads = 5

    simple_publisher.get_logger().info('NUM threads='+str(num_threads))

    executor = MultiThreadedExecutor(num_threads=num_threads)
    executor.add_node(simple_publisher)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        simple_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()