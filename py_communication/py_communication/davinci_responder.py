import rclpy
from rclpy.node import Node

import json
import time
import openai
import os

from std_msgs.msg import String

dialogue_template = """

Context: The SYSTEM is a small robot designed for human-robot interaction; and replies with short answers. It does not reply if it thinks the USER is not finished speaking. The OPERATOR is a manual human operator who can reply on behalf of the SYSTEM. Do not generate replies for the OPERATOR.

DIALOG START"""

class CommunicationSubscriber(Node):
	topic_name = "comm_topic"
	
	control_topic_name = "voice_recog_control_channel"

	enabled = True
	reenable_timestamp = 0

	def __init__(self):
		super().__init__('response_generator')
		self.input_subscription = self.create_subscription(
			String,
			self.topic_name,
			self.listener_callback,
			10)

		self.output_publisher = self.create_publisher(String, self.topic_name, 10)

		self.label_mappings = {
		"voice_recog" : "USER",
		"voice_recog_control" : "USER",
		"manual_reply" : "OPERATOR",
		self.get_name(): "SYSTEM",
		"focus_tracker": "SYSTEM"
		}
		self.reply_to = ["USER"]

		self.dialogue = dialogue_template
		print(self.dialogue,end="")
		

	def listener_callback(self, msg):
		dict = json.loads(msg.data)
		source = dict["source"]

		try:
			source_label = self.label_mappings[source] # this will eject into the except if its not in the mappings.

		except KeyError:
			print("recieved message from '{}', I don't know what this is. Discarding...".format(source))
			return


		if source_label == "":
			print("Ive been told to ignore messages sent from this source ('{}'). Discarding...".format(source))
			return
			
		# Note that we explicitly allow its own messages to get through here; as we use this to add messages to the dialog.
		dialogue_line = "\n\n{}: {}".format(source_label,dict["text"])
		print(dialogue_line,end="")
		self.dialogue += dialogue_line

		if source_label in self.reply_to:
			self.generateResponse()

	def generateResponse(self):
		# this is called when we want to use GPT to build upon the current dialogue state.
		self.dialogue += "\n\n{}: ".format(self.label_mappings[self.get_name()])

		response = openai.Completion.create(model = "text-davinci-003",prompt=self.dialogue,n=1,max_tokens=320)
		text = response["choices"][0]["text"]
		if text == "":
			return
		
		text = self.postProcess(text)
		#print("GPT came up with \"{}\"".format(text))

		dict = {}
		msg = String()
		dict["timestamp"] = time.time()
		dict["source"] = self.get_name()
		dict["text"] = text
		msg.data = json.dumps(dict,indent=4)
		self.output_publisher.publish(msg)

	def postProcess(self,text):
		text = text.replace("\n\nSYSTEM:","") # this is to stop it from generating "SYSTEM: SYSTEM: <text>" messages
		text = text.strip()
		return text

def main(args=None):
	rclpy.init(args=args)
	openai.api_key = os.environ["OPENAI_KEY"]
	comm_subscriber = CommunicationSubscriber()

	rclpy.spin(comm_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	comm_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()