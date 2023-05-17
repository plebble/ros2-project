from string_service_interface.srv import StringInStringOut

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(StringInStringOut, 'reverse_string', self.reverse_string_callback)

    def reverse_string_callback(self, request, response):
        response.response_data = request.request_data[::-1]
        self.get_logger().info("Recieved request to reverse the string '{}', will reply with '{}'".format(request.request_data,response.response_data))

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)
    print("service should be running")

    rclpy.shutdown()


if __name__ == '__main__':
    main()