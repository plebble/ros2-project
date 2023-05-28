import rclpy
from rclpy.node import Node

import json
import time

from std_msgs.msg import String


class MinimalPublisher(Node):
	topic_name = "comm_topic"
	def __init__(self):
		super().__init__('manual_reply')
		self.publisher_ = self.create_publisher(String, self.topic_name, 10)

	def send_message(self,text):
		msg = String()
		dict = {}
		dict["timestamp"] = time.time()
		dict["source"] = self.get_name()

		dict["text"] = text

		msg.data = json.dumps(dict,indent=4)
		self.publisher_.publish(msg)

		print(msg.data)

			


def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	# there are no callbacks here, the node does not need to spin.
	while True:
		text = input("what do you want the robot to say? :")
		minimal_publisher.send_message(text)
	
	# Shrek : 'as if thats ever gonnae happen'
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()