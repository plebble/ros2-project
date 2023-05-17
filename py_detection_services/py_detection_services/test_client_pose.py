import sys

from string_service_interface.srv import StringInStringOut #  same as before, needs to know the format of what goes in and out.
import rclpy
from rclpy.node import Node
import json
import base64
import cv2


def b64_jpg_encode(image):
    ret, jpg_encoded = cv2.imencode(".jpg",image)

    return base64.b64encode(jpg_encoded).decode()

class MinimalClientAsync(Node):

	def __init__(self):
		super().__init__('test_client')
		self.cli = self.create_client(StringInStringOut, 'detect_pose')
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
		self.req = StringInStringOut.Request()

	def send_request(self, string):
		self.req.request_data = string
		self.future = self.cli.call_async(self.req)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()


def main():
	rclpy.init()

	minimal_client = MinimalClientAsync()
	frame = cv2.imread(sys.argv[1])
	x1 = int(sys.argv[2])
	y1 = int(sys.argv[3])
	x2 = int(sys.argv[4])
	y2 = int(sys.argv[5])

	dict = {}
	dict["source"] = minimal_client.get_name()

	dict["image"] = {}
	dict["image"]["dimensions"] = list(frame.shape)
	dict["image"]["encoding"] = "b64_jpg"
	dict["image"]["data"] = str(b64_jpg_encode(frame))
	dict["bbox"] = [x1,y1,x2,y2]
	response = minimal_client.send_request(json.dumps(dict,indent=4))
	#minimal_client.get_logger().info("Input string: '{}'; Reversed string: '{}'".format(input_arg, response.response_data))
	minimal_client.get_logger().info("Recieved: \n {}".format(response.response_data))

	minimal_client.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()