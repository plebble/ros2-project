from string_service_interface.srv import StringInStringOut

import rclpy
from rclpy.node import Node
import json
import base64
import numpy as np
import cv2
import time
import torch

from SixDRepNet import SixDRepNet

def b64_jpg_decode(encoded):
	jpg_bytes = base64.b64decode(encoded)

	np_frame = np.frombuffer(jpg_bytes, dtype=np.uint8)
	return cv2.imdecode(np_frame,flags=1)

class MinimalService(Node):

	def __init__(self):
		super().__init__('pose_detector')
		self.srv = self.create_service(StringInStringOut, 'detect_pose', self.detect_faces_callback)

		print(torch.cuda.is_available())
		
		self.pose_detector = SixDRepNet()

	def detect_faces_callback(self, request, response):
		start = time.time()
		new_dict = {}
		new_dict["source"] = self.get_name()
		dict = json.loads(request.request_data)

		image = b64_jpg_decode(dict["image"]["data"])
		try:
			bbox = dict["bbox"]
			x1 = bbox[0]
			y1 = bbox[1]
			x2 = bbox[2]
			y2 = bbox[3]

			midpoint = (int((x2-x1)/2),int((y2-y1)/2)) # point in middle of bounding box
			smallest_dim = int(min(x2-x1,y2-y1))
			x_diff = (x2-x1) - smallest_dim
			y_diff = (y2-y1) - smallest_dim

			x1 += int(np.floor(x_diff/2))
			y1 += int(np.floor(y_diff/2))
			cropped_image = image[y1:y1+smallest_dim,x1:x1+smallest_dim]
			
		except KeyError:
			# if no bbox is supplied, run detection over whole image
			# as we assume its already cropped
			shape = np.shape(image)
			midpoint = (int(shape[1]/2),int(shape[0]/2))
			cropped_image = np.copy(image)

		pitch,yaw,roll = self.pose_detector.predict(cropped_image)
		new_dict["pose"] = [pitch.item(),yaw.item(),roll.item()]

		pitch = pitch.item() * np.pi / 180
		yaw = yaw.item() * np.pi / 180
		roll = roll.item() * np.pi / 180

		x = np.cos(pitch)*np.sin(-yaw)
		y = np.sin(-pitch)
		z = np.cos(pitch)*np.cos(yaw)
		new_dict["facing_vec"] = [x, y, z]
		new_dict["duration"] = round(time.time()-start,4)

		print(new_dict)

		response.response_data = json.dumps(new_dict,indent=4)
		self.get_logger().info("Recieved request from {}, have estimated pose as {}".format(dict["source"],new_dict["pose"]))
		print(response.response_data)

		return response


def main():
	rclpy.init()

	minimal_service = MinimalService()

	rclpy.spin(minimal_service)
	print("service should be running")

	rclpy.shutdown()


if __name__ == '__main__':
	main()