#! /usr/bin/env python3
from barista_interfaces.srv import TextCommand
import rclpy
from rclpy.node import Node


class FakePistonServer(Node):

    def __init__(self):

        super().__init__('fake_piston_service')
        self.piston_srv = self.create_service(TextCommand, '/piston_control', self.piston_callback)
        

    def piston_callback(self, request, response):
        direction = request.text

        if direction == "up" or direction == "down":
            self.get_logger().info('PISTON GOING ==='+str(direction))
            rclpy.sleep(5)
            self.get_logger().info('PISTON GOING ==='+str(direction)+"...DONE")
            response.success = True
            response.message = direction+"-finished"
        else:
            response.success = False
            response.message = "NOT SUPORTED direction="+str(direction)
               
        return response


def main(args=None):
    rclpy.init(args=args)
    fake_piston_server_obj = FakePistonServer()
    rclpy.spin(fake_piston_server_obj)
    rclpy.shutdown()


if __name__ == '__main__':
    main()