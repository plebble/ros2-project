import rclpy
from rclpy.node import Node

import cv2
import base64
import json
import time

from std_msgs.msg import String

def b64_jpg_encode(image):
    ret, jpg_encoded = cv2.imencode(".jpg",image)

    return base64.b64encode(jpg_encoded).decode()

class MinimalPublisher(Node):
    topic_name = "image_feed"
    target_res = (640,480)
    target_fps = 60
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(String, self.topic_name, 10)
        timer_period = 0.001
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.camera = cv2.VideoCapture(2) # should probably make this run with a parameter in the future
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH,self.target_res[0])
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT,self.target_res[1])                     

    def timer_callback(self):
        msg = String()
        dict = {}

        ret,frame = self.camera.read()
        dict["timestamp"] = time.time()
        dict["source"] = self.get_name()

        dict["image"] = {}
        dict["image"]["dimensions"] = list(frame.shape)
        dict["image"]["encoding"] = "b64_jpg"
        dict["image"]["data"] = str(b64_jpg_encode(frame))

        

        msg.data = json.dumps(dict,indent=4)
        self.publisher_.publish(msg)

        print(msg.data)


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