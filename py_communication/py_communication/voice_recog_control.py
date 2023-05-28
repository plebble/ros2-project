import rclpy
from rclpy.node import Node

import json
import time

from std_msgs.msg import String



class CommunicationSubscriber(Node):
	input_topic_name = "voice_recog_prefilter"
	output_topic_name = "comm_topic"
	control_topic_name = "voice_recog_control_channel"

	enabled = True
	reenable_timestamp = 0

	def __init__(self):
		super().__init__('tts_speech_node')
		self.input_subscription = self.create_subscription(
			String,
			self.input_topic_name,
			self.listener_callback,
			10)
		
		self.control_subscription = self.create_subscription(
			String,
			self.control_topic_name,
			self.control_callback,
			10)

		self.output_publisher = self.create_publisher(String, self.output_topic_name, 10)
		

	def listener_callback(self, msg):
		#self.get_logger().info("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		print("Enabled voice recog.? : {}".format(self.enabled))
		if self.enabled == False:
			print("Voice rec. currently disabled; discarding...")
			return
		else:
			print("Recieved message from '{}', parsing JSON...".format(self.input_topic_name))
		
		dict = json.loads(msg.data)
		if dict["listening_start"] < self.reenable_timestamp:
			print("Voice rec. started while the disabled; discarding...")
			return

		print("Allowing passthrough...")
		self.output_publisher.publish(msg)

	def control_callback(self,msg):
		text = msg.data
		if text == "enable":
			print("Recieved message to re-enable recognition...")
			self.enabled = True
			self.reenable_timestamp = time.time()

		if text == "disable":
			print("Recieved message to disable recognition...")
			self.enabled = False

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