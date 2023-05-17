import rclpy
from rclpy.node import Node
from string_service_interface.srv import StringInStringOut

import cv2
import base64
import json
import time
import numpy as np

from std_msgs.msg import String


def b64_jpg_decode(encoded):
	jpg_bytes = base64.b64decode(encoded)

	np_frame = np.frombuffer(jpg_bytes, dtype=np.uint8)
	return cv2.imdecode(np_frame,flags=1)

def b64_jpg_encode(image):
    ret, jpg_encoded = cv2.imencode(".jpg",image)

    return base64.b64encode(jpg_encoded).decode()

class Middleman(Node):
	topic_name = "image_feed"
	request_limit = 1 # how many requests the client is allowed to have waiting at a time

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

		self.client_futures = []
		

	def listener_callback(self, msg):
		# this will be called every time a message is obtained from a publisher
		print("Recieved message from '{}'".format(self.topic_name))
		
		dict = json.loads(msg.data)
		if dict["source"] == self.get_name():
			print("Recieved own message, ignoring...")
			return False
		
		if len(self.client_futures) < self.request_limit:
			print("Space available, making request...")
			request = StringInStringOut.Request()
			request.request_data = msg.data
			self.client_futures.append({"future":self.client.call_async(request),"request_data":request.request_data})
		else:
			print("No space in request buffer, ignoring message...")

	def process_response(self,response,request_data):
		dict = json.loads(response)
		detections = dict["detections"]
		if len(detections) == 0:
			return False
		
		req_dict = json.loads(request_data) 
		frame = b64_jpg_decode(req_dict["image"]["data"])
		for detection in detections:
			p1 = (detection["x1"],detection["y1"])
			p2 = (detection["x2"],detection["y2"])
			frame = cv2.rectangle(frame,p1,p2,(255,127,0),2)
			try:
				facing_vec = detection["facing_vec"]
				midpoint = (int(p1[0] + ((p2[0]-p1[0])/2)),int(p1[1] + ((p2[1]-p1[1])/2)))
				frame = cv2.arrowedLine(frame,midpoint,(int(midpoint[0]+facing_vec[0]*50),int(midpoint[1]+facing_vec[1]*50)),(0,0,255),5)
			except:
				pass

		msg = String()
		dict = {}

		dict["timestamp"] = time.time()
		dict["source"] = self.get_name()

		dict["image"] = {}
		dict["image"]["dimensions"] = list(frame.shape)
		dict["image"]["encoding"] = "b64_jpg"
		dict["image"]["data"] = str(b64_jpg_encode(frame))

		msg.data = json.dumps(dict,indent=4)
		self.publisher_.publish(msg)

	def spin(self):
		while rclpy.ok():
			rclpy.spin_once(self)
			incomplete_futures = []
			for future_dict in self.client_futures:
				future = future_dict["future"]
				if future.done():
					response = future.result().response_data
					print("Received response: {}".format(response))
					self.process_response(response,future_dict["request_data"])
				else:
					incomplete_futures.append(future_dict)

			self.client_futures = incomplete_futures
				
		



def main(args=None):
	rclpy.init(args=args)

	middleman = Middleman()

	middleman.spin()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	middleman.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()