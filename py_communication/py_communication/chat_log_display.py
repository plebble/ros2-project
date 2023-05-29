import rclpy
from rclpy.node import Node

import json
import time
from datetime import datetime
import pyttsx3

from std_msgs.msg import String



class CommunicationSubscriber(Node):
	topic_name = "comm_topic"
	def __init__(self):
		super().__init__('chat_log')
		self.subscription = self.create_subscription(
			String,
			self.topic_name,
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

	def listener_callback(self, msg):
		#self.get_logger().info("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		#print("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		dict = json.loads(msg.data)
		source = dict["source"]

		text = dict["text"]
		timestamp = dict["timestamp"]
		timestamp = datetime.fromtimestamp(timestamp).strftime("%H:%M:%S")

		print("[{}] <{}> {}".format(timestamp,source,text))

	def send_control_message(self,text):
		msg = String()
		msg.data = text
		self.publisher.publish(msg)

def main(args=None):
	rclpy.init(args=args)

	comm_subscriber = CommunicationSubscriber()

	rclpy.spin(comm_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	comm_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()