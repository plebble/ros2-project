import rclpy
from rclpy.node import Node

import cv2
import pyrealsense2 as rs
import numpy as np
import base64
import json
import time

from std_msgs.msg import String

def b64_jpg_encode(image):
	ret, jpg_encoded = cv2.imencode(".jpg",image)

	return base64.b64encode(jpg_encoded).decode()

def b64_typed_encode(frame): # this is intended for encoding 2D numpy arrays losslessly. returns a list with the element datatypes, encoded data
	return {"dtype":str(frame.dtype),"encoded":base64.b64encode(frame).decode("utf-8"),"shape":frame.shape}

class MinimalPublisher(Node):
	topic_name = "image_feed"
	target_res = (640,480)
	target_fps = 30
	def __init__(self):
		super().__init__('realsense_publisher')
		self.publisher_ = self.create_publisher(String, self.topic_name, 10)
		timer_period = 1/self.target_fps  # seconds, inverse of target framerate
		self.timer = self.create_timer(timer_period, self.timer_callback)
		self.i = 0

		self.pipeline = rs.pipeline()
		self.configuration = rs.config()

		self.configuration.enable_stream(rs.stream.color, self.target_res[0], self.target_res[1], rs.format.bgr8, self.target_fps)
		self.configuration.enable_stream(rs.stream.depth, self.target_res[0], self.target_res[1], rs.format.z16, self.target_fps)

		self.profile = self.pipeline.start(self.configuration)
		self.pipeline_started = True

		depth_scale = self.profile.get_device().first_depth_sensor().get_depth_scale()

		clipping_distance_in_meters = 0.1
		self.clipping_distance = clipping_distance_in_meters / depth_scale

		self.alignment = rs.align(rs.stream.color)
		

	def timer_callback(self):
		msg = String()
		dict = {}

		frame_pair = self.alignment.process(self.pipeline.wait_for_frames())
		colour_frame = np.asanyarray(frame_pair.get_color_frame().get_data())
		depth_frame = np.asanyarray(frame_pair.get_depth_frame().get_data())

		#colour_frame = cv2.applyColorMap(cv2.convertScaleAbs(depth_frame, alpha=0.03), cv2.COLORMAP_JET)

		dict["timestamp"] = time.time()
		dict["source"] = self.get_name()

		dict["image"] = {}
		dict["image"]["dimensions"] = list(colour_frame.shape)
		dict["image"]["encoding"] = "b64_jpg"
		dict["image"]["data"] = str(b64_jpg_encode(colour_frame))

		dict["depth"] = {}
		dict["depth"]["dimensions"] = list(depth_frame.shape)
		dict["depth"]["encoding"] = "b64_typed"
		dict["depth"]["data"] = b64_typed_encode(depth_frame)		

		msg.data = json.dumps(dict,indent=4)
		self.publisher_.publish(msg)

		print(msg.data)
		# self.get_logger().info(msg.data)
		self.i += 1


def main(args=None):
	rclpy.init(args=args)

	minimal_publisher = MinimalPublisher()

	rclpy.spin(minimal_publisher)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	minimal_publisher.camera.release()
	minimal_publisher.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()