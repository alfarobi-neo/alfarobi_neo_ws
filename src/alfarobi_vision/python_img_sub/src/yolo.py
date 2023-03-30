import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

import cv2
from cv_bridge import CvBridge, CvBridgeError 

import numpy
from typing import Tuple, Iterator

from yolov5s.annotate_v import annotate, parse_args

class VisionLoader:
	def __init__(self, args):
		rospy.init_node('python_image_sub', anonymous=True)
	
		self.bridge = CvBridge()

		self.args = args

		self.mat = None

		self.sub = rospy.Subscriber('/img_source_node/image_src', Image, self.callback)
		
		self.ball_pub = rospy.Publisher('ball_distance', Float32, queue_size=10)

	def __iter__(self) -> Iterator[Tuple[numpy.ndarray, numpy.ndarray]]:
		while True:
			if self.mat is not None:
				frame = cv2.flip(self.mat, 1) 
				img_resized = cv2.resize(frame, self.args.image_shape)
				img_transposed = img_resized[:, :, ::-1].transpose(2, 0, 1)
				yield img_transposed, frame

	def callback(self, img_msg):
		try:
			mat = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
			self.mat = cv2.cvtColor(mat, cv2.COLOR_BGR2RGB)
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))

	def __call__(self):
		annotate(self.args, self)

def main():
	args = parse_args()
	program = VisionLoader(args)
	program()

if __name__ == '__main__':
	main()
