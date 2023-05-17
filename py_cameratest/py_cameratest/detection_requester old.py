import rclpy
from rclpy.node import Node
from string_service_interface.srv import StringInStringOut

import cv2
import base64
import json
import time
import numpy as np

from std_msgs.msg import String


class Middleman(Node):
	topic_name = "image_feed"
	active = False
	def __init__(self):
		super().__init__('detection_requester')
		self.subscription = self.create_subscription(
			String,
			self.topic_name,
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

		self.publisher_ = self.create_publisher(String, self.topic_name, 10)

		self.client = self.create_client(StringInStringOut, 'detect_faces')
		while not self.client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.req = StringInStringOut.Request()
		

	def send_request(self, string):
		self.req.request_data = string
		print("reach 1")
		self.active = True
		self.future = self.client.call_async(self.req)
		print("reach 2")
		rclpy.spin_until_future_complete(self, self.future)
		print("reach 3")
		self.active = False
		return self.future.result()

	def listener_callback(self, msg):
		# this will be called every time a message is obtained from a publisher
		print("Recieved message from '{}'".format(self.topic_name))
		
		dict = json.loads(msg.data)
		if dict["source"] == self.get_name():
			return False
		
		if self.active == False:
			response = self.send_request(msg.data)

			print(response.response_data)
		



def main(args=None):
	rclpy.init(args=args)

	middleman = Middleman()

	rclpy.spin(middleman)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	middleman.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()