from string_service_interface.srv import StringInStringOut

import rclpy
from rclpy.node import Node
import json
import base64
import numpy as np
import cv2
import time

from mtcnn import MTCNN

def b64_jpg_decode(encoded):
    jpg_bytes = base64.b64decode(encoded)

    np_frame = np.frombuffer(jpg_bytes, dtype=np.uint8)
    return cv2.imdecode(np_frame,flags=1)

class MinimalService(Node):

	def __init__(self):
		super().__init__('mtcnn_face_detector')
		self.srv = self.create_service(StringInStringOut, 'detect_faces', self.detect_faces_callback)
		
		self.face_detector = MTCNN()

	def detect_faces_callback(self, request, response):
		start = time.time()
		new_dict = {}
		new_dict["source"] = self.get_name()
		dict = json.loads(request.request_data)

		image = b64_jpg_decode(dict["image"]["data"])
		detections = self.face_detector.detect_faces(image)
		new_dict["detections"] = detections

		new_dict["duration"] = round(time.time()-start,4)
		response.response_data = json.dumps(new_dict,indent=4)
		self.get_logger().info("Recieved request from {}, have made {} detections".format(dict["source"],len(detections)))
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