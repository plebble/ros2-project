import rclpy
from rclpy.node import Node
from string_service_interface.srv import StringInStringOut

import cv2
import base64
import json
import time
import uuid
import numpy as np

from std_msgs.msg import String


def b64_jpg_decode(encoded):
	jpg_bytes = base64.b64decode(encoded)

	np_frame = np.frombuffer(jpg_bytes, dtype=np.uint8)
	return cv2.imdecode(np_frame,flags=1)

def b64_jpg_encode(image):
	ret, jpg_encoded = cv2.imencode(".jpg",image)

	return base64.b64encode(jpg_encoded).decode()

def clamp(val,s,b):
    return max(s,min(val,b))


import uuid
import time

class FaceTracker:
	expiry_timer = 5
	max_correlation_distance = 200
	def __init__(self):
		self.tracked_faces = {}

	def update(self,detections):
		temp = {}
		for face in detections:
			uuid_id=uuid.uuid4()
			id = str(uuid_id)
			np.random.seed(int(uuid_id) & (1<<32)-1)
			np_colour = np.random.randint(0,256,size=3,dtype=np.uint8)
			colour = [int(np_colour[0]),int(np_colour[1]),int(np_colour[2])]
			center = (int((face["x1"] + face["x2"]) / 2),int((face["y1"] + face["y2"]) / 2))
			last_updated = time.time()
			temp[id] = {"center":center,
						"last_updated":last_updated,
						"detection_data":face,
						"persistent":{"colour":colour}}
			
		detections = temp.copy()
		del temp


		distance_pairings = {}
		for existing_id in list(self.tracked_faces.keys()):
			existing_box_center = self.tracked_faces[existing_id]["center"]
			for new_id in list(detections.keys()):
				new_center = detections[new_id]["center"]

				dx = existing_box_center[0]-new_center[0]
				dy = existing_box_center[1]-new_center[1]
				distance = (dx**2+dy**2)**0.5
				distance_pairings[(existing_id,new_id)] = distance

		# I have no idea what this does or how it works; but it sorts the distance pairings by the distance
		distance_pairings = {k: v for k, v in sorted(distance_pairings.items(), key=lambda item: item[1])}

		temp = {}

		for pairing in list(distance_pairings.keys()):
			if len(detections) == 0 or len(self.tracked_faces) == 0:
				break

			if distance_pairings[pairing] > self.max_correlation_distance:
				break

			existing_id = pairing[0]
			new_id = pairing[1]
			try:
				self.tracked_faces[existing_id]
				updated_info = detections[new_id]
			except KeyError:
				# this happens when either the new face or the old face has already been paired up.
				continue
			
			try:
				updated_info["persistent"] = self.tracked_faces[existing_id]["persistent"]
			except KeyError:
				pass
			temp[existing_id] = updated_info
			del detections[new_id]
			del self.tracked_faces[existing_id]

		for existing_id in list(self.tracked_faces.keys()):
			# now we cull the ones that havent been updated, if they have expired
			
			if self.tracked_faces[existing_id]["last_updated"] + self.expiry_timer < time.time():
				del self.tracked_faces[existing_id]

		temp.update(self.tracked_faces)
		temp.update(detections)
		self.tracked_faces = temp.copy()
		




class TrackerNode(Node):
	topic_name = "image_feed"
	control_topic_name = "voice_recog_control_channel"
	dialog_topic_name = "comm_topic"
	request_limit = 1 # how many requests the client is allowed to have waiting at a time


	def __init__(self):
		super().__init__('focus_tracker')
		self.subscription = self.create_subscription(
			String,
			self.topic_name,
			self.listener_callback,
			10)
		self.subscription  # prevent unused variable warning

		self.image_publisher = self.create_publisher(String, self.topic_name, 10)
		self.control_publisher = self.create_publisher(String, self.control_topic_name, 10)
		self.dialog_publisher = self.create_publisher(String, self.dialog_topic_name, 10)

		self.send_control_message("disable") # when we're using this; we want it to start disabled until we get focus

		self.client = self.create_client(StringInStringOut, 'detect_faces')

		self.client_futures = []

		self.face_tracker = FaceTracker()
		self.focused_id = ""

		self.focus_lock_cosang_threshold = 0.95
		self.focus_lock_duration = 2 # change this once finished testing

		self.focus_unlock_cosang_threshold = 0.90
		self.focus_unlock_duration = 5

		

	def listener_callback(self, msg):
		#print("Recieved message on '{}'".format(self.topic_name))
		dict = json.loads(msg.data)
		if dict["source"] == self.get_name():
			# this node publishes to 'image_feed' for debug/display purposes
			#print("Recieved own message, ignoring...")
			return False
		
		self.current_frame = b64_jpg_decode(dict["image"]["data"])	
		
		if len(self.client_futures) < self.request_limit:
			#print("Space available, making request...")
			request = StringInStringOut.Request()
			request.request_data = msg.data
			self.client_futures.append({"future":self.client.call_async(request),"request_data":request.request_data})
		else:
			#print("No space in request buffer, ignoring message...")
			return False

	def process_response(self,response,request_data):
		dict = json.loads(response)
		detections = dict["detections"]
		
		self.face_tracker.update(detections)
		#print(json.dumps(self.face_tracker.tracked_faces,indent=4))

		#start by pruning the focused_id if it doesnt exist
		if self.focused_id != "":
			try:
				self.face_tracker.tracked_faces[self.focused_id]
			except:
				self.unfocused_action()
				self.focused_id = ""
			
		self.draw_debug()

		if self.focused_id == "":
			print("don't have lock, trying to find it...")
			for id in list(self.face_tracker.tracked_faces.keys()):
				try:
					facing_vec = self.face_tracker.tracked_faces[id]["detection_data"]["facing_vec"]
				except KeyError:
					continue

				# due to the way the maths works, the cos(ang) from the facing direction to the camera is just the z value. ang= arccos(z) but we dont need the angle
				cos_ang = facing_vec[2]
				if cos_ang >= self.focus_lock_cosang_threshold:
					try:
						# if its already started locking on, dont change the timestamp
						self.face_tracker.tracked_faces[id]["persistent"]["focus_lock_started"]
					except KeyError:
						# if we've only just started locking onto this face, then mark the timestamp
						self.face_tracker.tracked_faces[id]["persistent"]["focus_lock_started"] = time.time()
				else:
					try:
						del self.face_tracker.tracked_faces[id]["persistent"]["focus_lock_started"]
					except:
						pass
					# we know that this candidate isnt near the lockon, so we can continue here
					continue
				
				lock_duration = time.time() - self.face_tracker.tracked_faces[id]["persistent"]["focus_lock_started"]
				if lock_duration > self.focus_lock_duration:
					# now we start focusing on this person
					for other_id in list(self.face_tracker.tracked_faces.keys()):
						try:
							del self.face_tracker.tracked_faces[other_id]["persistent"]["focus_lock_started"]
						except:
							pass
					self.focused_id = id
					self.focused_action()
					return
		

		if self.focused_id != "":
			print(json.dumps(self.face_tracker.tracked_faces[self.focused_id],indent=4))
			try:
				facing_vec = self.face_tracker.tracked_faces[self.focused_id]["detection_data"]["facing_vec"]
			except KeyError:
				return
			
			cos_ang = facing_vec[2]
			if cos_ang <= self.focus_unlock_cosang_threshold:
				try:
					self.face_tracker.tracked_faces[self.focused_id]["persistent"]["focus_unlock_started"]
				except:
					self.face_tracker.tracked_faces[self.focused_id]["persistent"]["focus_unlock_started"] = time.time()

				unlock_duration = time.time() - self.face_tracker.tracked_faces[self.focused_id]["persistent"]["focus_unlock_started"]
				if unlock_duration > self.focus_unlock_duration:
					del self.face_tracker.tracked_faces[self.focused_id]["persistent"]["focus_unlock_started"]
					self.unfocused_action()
					self.focused_id = ""
			else:
				try:
					del self.face_tracker.tracked_faces[self.focused_id]["persistent"]["focus_unlock_started"]
				except:
					pass
		

	def spin(self):
		while rclpy.ok():
			rclpy.spin_once(self)
			incomplete_futures = []
			for future_dict in self.client_futures:
				future = future_dict["future"]
				if future.done():
					response = future.result().response_data
					#print("Received response: {}".format(response))
					self.process_response(response,future_dict["request_data"])
				else:
					incomplete_futures.append(future_dict)

			self.client_futures = incomplete_futures

	def unfocused_action(self):
		print("unfocusing from {}...{}".format(self.focused_id[:4],self.focused_id[-4:]))
		self.send_dialog_message("Goodbye!")
		time.sleep(1.5)
		self.send_control_message("disable")

	def focused_action(self):
		print("focusing on {}...{}".format(self.focused_id[:4],self.focused_id[-4:]))
		self.send_dialog_message("Hello, I'am SYSTEM, how may I help you today?")
		self.send_control_message("enable")
	
	def send_control_message(self,text):
		msg = String()
		msg.data = text
		self.control_publisher.publish(msg)

	def send_dialog_message(self,text):
		dict = {}
		msg = String()
		dict["timestamp"] = time.time()
		dict["source"] = self.get_name()
		dict["text"] = text
		msg.data = json.dumps(dict,indent=4)
		self.dialog_publisher.publish(msg)

	def draw_debug(self):
		frame = self.current_frame
		height,width,chan = frame.shape
		for id in list(self.face_tracker.tracked_faces.keys()):
			detection = self.face_tracker.tracked_faces[id]["detection_data"]
			persistent = self.face_tracker.tracked_faces[id]["persistent"]
			x1 = clamp(detection["x1"],0,width)
			y1 = clamp(detection["y1"],0,height)
			x2 = clamp(detection["x2"],0,width)
			y2 = clamp(detection["y2"],0,height)

			colour = tuple(persistent["colour"])
			print(colour)

			frame = cv2.rectangle(frame,(x1,y1),(x2,y2),colour,2)
			frame = cv2.putText(frame,"{}...{}".format(id[:4],id[-4:]),(x1+5,y1+20),cv2.FONT_HERSHEY_SIMPLEX,float((x2-x1)*0.005),colour,2)

			try:
				duration=time.time()-self.face_tracker.tracked_faces[id]["persistent"]["focus_lock_started"]
				progress = duration / self.focus_lock_duration
				box_width = int(abs(x2-x1) * progress)

				frame = cv2.rectangle(frame,(x1,y2-20),(x1+box_width,y2),(0,200,0),-1)

			except KeyError:
				pass

			try:
				duration=time.time()-self.face_tracker.tracked_faces[id]["persistent"]["focus_unlock_started"]
				progress = duration / self.focus_unlock_duration
				box_width = int(abs(x2-x1) * progress)

				frame = cv2.rectangle(frame,(x1,y2-20),(x1+box_width,y2),(0,0,200),-1)

			except KeyError:
				pass

			
			if id == self.focused_id:
				frame = cv2.putText(frame,"Locked onto: {}...{}".format(id[:4],id[-4:]),(10,30),cv2.FONT_HERSHEY_SIMPLEX,float(width*0.001),colour,2)
			

		msg = String()
		dict = {}
		dict["timestamp"] = time.time()
		dict["source"] = self.get_name()
		dict["image"] = {}
		dict["image"]["dimensions"] = list(frame.shape)
		dict["image"]["encoding"] = "b64_jpg"
		dict["image"]["data"] = str(b64_jpg_encode(frame))

		msg.data = json.dumps(dict,indent=4)
		self.image_publisher.publish(msg)



def main(args=None):
	rclpy.init(args=args)

	node = TrackerNode()

	node.spin()

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	node.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()