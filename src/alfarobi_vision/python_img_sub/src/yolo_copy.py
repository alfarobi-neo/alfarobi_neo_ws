import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError 

import numpy
from typing import Tuple, Iterator

from yolov5s.annotate import annotate
from yolov5s.annotate import parse_args

class Main:
	def __init__(self, img_size:Tuple[int,int]=(416,416)):
		rospy.init_node('python_image_sub', anonymous=True)
	
		self.bridge = CvBridge()

		self.image_size = img_size
		self.mat = None

		self.sub = rospy.Subscriber('/camera/image_raw', Image, self.callback)

	def __iter__(self) -> Iterator[Tuple[numpy.ndarray, numpy.ndarray]]:
			while True:
				if self.mat is not None:
					frame = cv2.flip(self.mat, 1) 
					img_resized = cv2.resize(frame, self.image_size)
					img_transposed = img_resized[:, :, ::-1].transpose(2, 0, 1)
					yield img_transposed, frame

	def callback(self, img_msg):
		try:
			self.mat = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))

	def __call__(self):
		args = parse_args()
		annotate(args, self)

if __name__ == '__main__':
	main = Main()
	main()

