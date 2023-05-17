import rclpy
from rclpy.node import Node

import cv2
import base64
import json
import time
import numpy as np

from std_msgs.msg import String

def show_img(label,image):
	cv2.imshow(label,image)
	cv2.waitKey(1)

def b64_jpg_decode(encoded):
    jpg_bytes = base64.b64decode(encoded)

    np_frame = np.frombuffer(jpg_bytes, dtype=np.uint8)
    return cv2.imdecode(np_frame,flags=1)

def b64_typed_decode(encoded):
	dtype = np.dtype(encoded["dtype"])
	temp = np.frombuffer(base64.decodestring(bytearray(encoded["encoded"],"utf-8")), dtype)
	return temp.reshape(encoded["shape"])


class MinimalSubscriber(Node):
	topic_name = "image_feed"
	def __init__(self):
		super().__init__('minimal_subscriber')
		self.subscription = self.create_subscription(
			String,
			self.topic_name,
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

	def listener_callback(self, msg):
		#self.get_logger().info("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		print("Recieved message from '{}', parsing JSON...".format(self.topic_name))
		dict = json.loads(msg.data) 


		frame = b64_jpg_decode(dict["image"]["data"])	

		

		if dict["source"] == "realsense_publisher": # this can be done either by the source label, or a try-except block of data["depth"]
			depth_frame = b64_typed_decode(dict["depth"]["data"])
			colour_map = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)

			frame = np.concatenate((frame, colour_map), axis=1)


		show_img(dict["source"], frame)
		start = dict["timestamp"]
		end = time.time()
		duration = end-start
		print("duration for transmission:",duration)
		print("transmission FPS:",1/duration)


def main(args=None):
	rclpy.init(args=args)

	minimal_subscriber = MinimalSubscriber()

	rclpy.spin(minimal_subscriber)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()