import sys

from string_service_interface.srv import StringInStringOut #  same as before, needs to know the format of what goes in and out.
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

	def __init__(self):
		super().__init__('minimal_client_async')
		self.cli = self.create_client(StringInStringOut, 'reverse_string')
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.req = StringInStringOut.Request()

	def send_request(self, string):
		self.req.request_data = string
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()


def main():
	rclpy.init()

	minimal_client = MinimalClientAsync()
	input_arg = sys.argv[1]
	response = minimal_client.send_request(input_arg)
	minimal_client.get_logger().info("Input string: '{}'; Reversed string: '{}'".format(input_arg, response.response_data))

	minimal_client.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()