from string_service_interface.srv import StringInStringOut

import rclpy
from rclpy.node import Node
import json
import base64
import numpy as np
import cv2
import time
import torch

from facenet_pytorch import MTCNN

def b64_jpg_decode(encoded):
    jpg_bytes = base64.b64decode(encoded)

    np_frame = np.frombuffer(jpg_bytes, dtype=np.uint8)
    return cv2.imdecode(np_frame,flags=1)

class MinimalService(Node):

	def __init__(self):
		super().__init__('mtcnn_face_detector_torch')
		self.srv = self.create_service(StringInStringOut, 'detect_faces', self.detect_faces_callback)

		print(torch.cuda.is_available())
		
		self.face_detector = MTCNN(select_largest=False,device='cuda')

	def detect_faces_callback(self, request, response):
		start = time.time()
		new_dict = {}
		new_dict["source"] = self.get_name()
		dict = json.loads(request.request_data)

		image = b64_jpg_decode(dict["image"]["data"])
		detection = self.face_detector.detect(image)
		detections = []
		try:
			n = len(detection[0])
			
			for i in range(n):
				det_dict = {}
				det_dict["confidence"] = float(detection[1][i])
				bbox = detection[0][i]

				det_dict["x1"] = int(bbox[0])
				det_dict["y1"] = int(bbox[1])
				det_dict["x2"] = int(bbox[2])
				det_dict["y2"] = int(bbox[3])

				detections.append(det_dict)


			new_dict["detections"] = detections
		except TypeError:
			new_dict["detections"] = []

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