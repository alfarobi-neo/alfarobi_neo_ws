#!/usr/bin/env python3
import rospy 
# import os
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import cv2
from cv_bridge import CvBridge, CvBridgeError 

import numpy
from typing import Tuple, Iterator

from yolov5s.annotate_v import annotate, parse_args

class VisionLoader:
	def __init__(self, args):
		rospy.init_node('python_image_sub', anonymous=True)
	
		self.bridge = CvBridge()
		# self.rate = rospy.Rate(30)

		self.args = args

		self.mat = None

		self.img_sub = rospy.Subscriber('/img_source_node/image_src', Image, self.imageCallback)
		
		self.ball_pos_pub = rospy.Publisher('/python_image_sub/ball_pos', Point, queue_size=100)

	def __iter__(self) -> Iterator[Tuple[numpy.ndarray, numpy.ndarray]]:
		while True:
			if self.mat is not None: 
				frame = self.mat
				# frame = cv2.flip(self.mat, 1) 
				img_resized = cv2.resize(frame, self.args.image_shape)
				img_transposed = img_resized[:, :, ::-1].transpose(2, 0, 1)
				yield img_transposed, frame

	def publish_(self, x:int, y:int, z=-1):
		coordinate = Point(x,y,z)
		self.ball_pos_pub.publish(coordinate)

	def imageCallback(self, img_msg):
		try:
			mat = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
			self.mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))

	def __call__(self):
		# print(os.getcwd())
		annotate(self.args, self)

def main():
	args = parse_args()
	program = VisionLoader(args)
	program()

if __name__ == '__main__':
	main()
