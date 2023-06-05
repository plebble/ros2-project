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
		super().__init__('voice_recog')
		self.publisher = self.create_publisher(String, self.topic_name, 10)

		self.audio_engine = sr.Recognizer()
		self.audio_engine.dynamic_energy_threshold = False
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
	mic_label = "USB PnP"
	# there are no callbacks here, the node does not need to spin.
	
	with find_mic(mic_label) as source:
		minimal_publisher.audio_engine.dynamic_energy_threshold = False
		print("calibrating microphone, please be quiet!")
		minimal_publisher.audio_engine.adjust_for_ambient_noise(source,duration=3)
		minimal_publisher.audio_engine.energy_threshold *= 5
		print("calibration finished, calibrated as {}.".format(minimal_publisher.audio_engine.energy_threshold))
		while True:
			minimal_publisher.audio_engine.energy_threshold = 180 # force it
			dict = {}
			dict["listening_start"] = time.time()
			print("started listening for something to be said (threshold = {:.2f})...".format(minimal_publisher.audio_engine.energy_threshold))
			try:
				audio = minimal_publisher.audio_engine.listen(source,timeout=10)
			except sr.WaitTimeoutError:
				continue
			
			print("stopped listening, sending audio... (timestamp = {:.4f})".format(time.time()))
			try:
				text = minimal_publisher.audio_engine.recognize_google(audio)
				print("speech recognised as:",text)
			except sr.UnknownValueError:
				print("could not recognise anything.")
				continue
				
			msg = String()
			dict["timestamp"] = time.time()
			dict["source"] = minimal_publisher.get_name()
			dict["text"] = text
			msg.data = json.dumps(dict,indent=4)
			minimal_publisher.publisher.publish(msg)

			print(msg.data)
				
	
	# Shrek : 'as if thats ever gonnae happen'
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()