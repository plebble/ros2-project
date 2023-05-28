import rclpy
from rclpy.node import Node

import json
import time
import speech_recognition as sr

from std_msgs.msg import String

def find_mic(device_label):
	for index, name in enumerate(sr.Microphone.list_microphone_names()):
		print("\"{1}\" : {0}".format(index, name))
		if device_label in name:
			print("Found mic with target name")
			mic = sr.Microphone(device_index=index)
			break
	else:
		print("Could not find specified microphone, using default")
		mic = sr.Microphone()

	return mic

class MinimalPublisher(Node):
	topic_name = "voice_recog_prefilter"
	def __init__(self):
		super().__init__('voice_recognition')
		self.publisher = self.create_publisher(String, self.topic_name, 10)

		self.r = sr.Recognizer()
		self.mic = find_mic("Arctis Pro Wireless: USB Audio")

	def send_message(self,text):
		msg = String()
		dict = {}
		dict["timestamp"] = time.time()
		

		msg.data = json.dumps(dict,indent=4)
		self.publisher.publish(msg)

		print(msg.data)

			


def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	# there are no callbacks here, the node does not need to spin.
	with minimal_publisher.mic as source:
		minimal_publisher.r.adjust_for_ambient_noise(source)
		while True:
			try:
				msg = String()
				dict = {}

				dict["listening_start"] = time.time()
				print("started listening for something to be said...")
				audio = minimal_publisher.r.listen(source)

				print("stopped listening, sending audio...")
				text = minimal_publisher.r.recognize_google(audio)
				print("speech recognised as:",text)

				dict["timestamp"] = time.time()
				dict["source"] = minimal_publisher.get_name()
				dict["text"] = text

				msg.data = json.dumps(dict,indent=4)
				minimal_publisher.publisher.publish(msg)

				print(msg.data)
			except sr.UnknownValueError:
				continue
	
	# Shrek : 'as if thats ever gonnae happen'
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()