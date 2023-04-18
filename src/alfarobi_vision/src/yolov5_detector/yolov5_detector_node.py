#!/usr/bin/env python3
import rospy 
# import os
import sys
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

import cv2
from cv_bridge import CvBridge, CvBridgeError 

import numpy
from typing import Tuple, Iterator

from object_detection.annotate_v import annotate, parse_args

class VisionLoader:
	def __init__(self, args):
		rospy.init_node('yolov5_detector_node', anonymous=True)
	
		self.bridge = CvBridge()
		# self.rate = rospy.Rate(30)

		self.program_stop = False
		self.stop_time = None
		self.args = args
		self.mat = None
		self.img_sub = rospy.Subscriber('/img_source_node/image_src', Image, self.imageCallback)
		self.ball_pos_pub = rospy.Publisher('/yolov5_detector_node/ball_pos', Point, queue_size=100)

	def __iter__(self) -> Iterator[Tuple[numpy.ndarray, numpy.ndarray]]:
		while True:
			if self.mat is not None: 
				frame = self.mat
				# frame = cv2.flip(self.mat, 1) 
				img_resized = cv2.resize(frame, self.args.image_shape)
				img_transposed = img_resized[:, :, ::-1].transpose(2, 0, 1)
				yield img_transposed, frame
			elif self.program_stop:
				break

	def publishPos(self, x:int, y:int, z=-1):
		coordinate = Point(x,y,z)
		self.ball_pos_pub.publish(coordinate)

	def imageCallback(self, img_msg):
		try:
			mat = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
			self.mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))

	def __call__(self):
		annotate(self.args, self)

	def suddenStop(self):
		cv2.destroyAllWindows()
		self.program_stop = True
		sys.exit(1)

def main():
	args = parse_args()
	program = VisionLoader(args)
	program()

if __name__ == '__main__':
	main()
