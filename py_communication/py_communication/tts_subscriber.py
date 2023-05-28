import rclpy
from rclpy.node import Node

import json
import time
import pyttsx3

from std_msgs.msg import String



class CommunicationSubscriber(Node):
	topic_name = "comm_topic"
	control_topic_name = "voice_recog_control_channel"
	def __init__(self):
		super().__init__('tts_speech_node')
		self.subscription = self.create_subscription(
			String,
			self.topic_name,
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

		self.publisher = self.create_publisher(String, self.control_topic_name, 10)
		
		self.tts_engine = pyttsx3.init()

	def listener_callback(self, msg):
		#self.get_logger().info("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		print("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		dict = json.loads(msg.data)
		print(msg.data)
		if dict["source"] == "voice_recognition":
			print("Recieved voice_recognition; not speaking because it is me.")
			# dont say anything that the voice recog. picked up, the robot doesnt need to say what it hears
			return

		text = dict["text"]

		print("Sending message to disable voice recognition, so it doesn't pick up the tts.")
		self.send_control_message("disable")

		print("Using TTS to say: {}".format(text))
		if text == "":
			return

		self.tts_engine.say(text)
		self.tts_engine.runAndWait()
		self.send_control_message("enable")

		start = dict["timestamp"]
		end = time.time()
		duration = end-start
		print("duration to recieve and talk: {:.2f}".format(duration))

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