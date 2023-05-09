#!/usr/bin/env python3

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge, CvBridgeError 

import os
import cv2
import numpy as np
import importlib.util
import rospy

class VisionLoader:
	def __init__(self):   
		rospy.init_node('tflite_detector_node', anonymous=True)
		self.ball_pos_pub = rospy.Publisher("ball_pos", Point, queue_size=100)
		self.bridge = CvBridge()
		self.img_sub = rospy.Subscriber('/img_source_node/image_src', Image, self.imageCallback)
		
	def publishPos(self, x:int, y:int, z = -1):
		coordinate = Point(x, y, z)
		self.ball_pos_pub.publish(coordinate)
		
	def update(self):
		# Keep looping indefinitely until the thread is stopped
		while True:
			# If the camera is stopped, stop the thread
			if self.stopped:
				# Close camera resources
				self.img_sub.unregister()
				return

			# Otherwise, grab the next frame from the stream
			if self.frame is not None :
				frame = self.frame
				self.frame = None
				
	# Do some processing on the frame here
	def imageCallback(self, msg):
		# Convert ROS image to OpenCV image
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, "passthrough")
			self.frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	
		except CvBridgeError as e:
			rospy.logerr("CvBridge Error: {0}".format(e))

class Argumen:
	def __init__(self) -> None:
		self.modeldir = "/home/syauqinadhif/alfarobi_backup/src/alfarobi_vision/src/tflite_detector/datasets"
		self.graph = "detect.tflite"
		self.labels = "labelmap.txt"
		self.threshold = 0.7
		self.resolution = "640x480"
		self.edgetpu = None

  
main = VisionLoader()
args = Argumen()

MODEL_NAME = args.modeldir
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
#edge TPU auto nyala
use_TPU = False

# Import TensorFlow libraries
# If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
# If using Coral Edge TPU, import the load_delegate library
pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
	from tflite_runtime.interpreter import Interpreter
	if use_TPU:
		from tflite_runtime.interpreter import load_delegate
else:
	from tensorflow.lite.python.interpreter import Interpreter
	if use_TPU:
		from tensorflow.lite.python.interpreter import load_delegate

# If using Edge TPU, assign filename for Edge TPU model
if use_TPU:
	# If user has specified the name of the .tflite file, use that name, otherwise use default 'edgetpu.tflite'
	if (GRAPH_NAME == 'detect.tflite'):
		GRAPH_NAME = 'edgetpu.tflite'       

# Get path to current working directory
CWD_PATH = os.getcwd()

# Path to .tflite file, which contains the model that is used for object detection
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

# Path to label map file
PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

# Load the label map
with open(PATH_TO_LABELS, 'r') as f:
	labels = [line.strip() for line in f.readlines()]

# Have to do a weird fix for label map if using the COCO "starter model" from
# https://www.tensorflow.org/lite/models/object_detection/overview
# First label is '???', which has to be removed.
if labels[0] == '???':
	del(labels[0])

# Load the Tensorflow Lite model.
# If using Edge TPU, use special load_delegate argument
if use_TPU:
	interpreter = Interpreter(model_path=PATH_TO_CKPT,
							  experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
	print(PATH_TO_CKPT)
else:
	interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Check output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if ('StatefulPartitionedCall' in outname): # This is a TF2 model
	boxes_idx, classes_idx, scores_idx = 1, 3, 0
else: # This is a TF1 model
	boxes_idx, classes_idx, scores_idx = 0, 1, 2

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

while True:

	# Start timer (for calculating frame rate)
	t1 = cv2.getTickCount()

	# Grab frame from video stream
	# frame1 = videostream.read()
	frame1 = main.frame

	# Acquire frame and resize to expected shape [1xHxWx3]
	frame = frame1.copy()
	frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	frame_resized = cv2.resize(frame_rgb, (width, height))
	input_data = np.expand_dims(frame_resized, axis=0)

	# Normalize pixel values if using a floating model (i.e. if model is non-quantized)
	if floating_model:
		input_data = (np.float32(input_data) - input_mean) / input_std

	# Perform the actual detection by running the model with the image as input
	interpreter.set_tensor(input_details[0]['index'],input_data)
	interpreter.invoke()

	# Retrieve detection results
	boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
	classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
	scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

	# Loop over all detections and draw detection box if confidence is above minimum threshold
	for i in range(len(scores)):
		if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):
			# Get bounding box coordinates and draw box
			# Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
			ymin = int(max(1,(boxes[i][0] * imH)))
			xmin = int(max(1,(boxes[i][1] * imW)))
			ymax = int(min(imH,(boxes[i][2] * imH)))
			xmax = int(min(imW,(boxes[i][3] * imW)))
			
			cv2.rectangle(frame1, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

			centerx = int((xmin+xmax)/2)
			centery = int((ymax+ymin)/2)

			cv2.circle(frame1,(centerx,centery),2,(255,255,255),-1)

			main.publishPos(centerx, centery)
			
			# Draw label
			object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
			label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
			labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
			label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
			cv2.rectangle(frame1, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
			cv2.putText(frame1, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

	# Draw framerate in corner of frame
	cv2.putText(frame1,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

	# All the results have been drawn on the frame, so it's time to display it.
	cv2.imshow('Object detector', frame1)

	# Calculate framerate
	t2 = cv2.getTickCount()
	time1 = (t2-t1)/freq
	frame_rate_calc= 1/time1

	# Press 'q' to quit
	if cv2.waitKey(1) == ord('q'):
		break

# Clean up
cv2.destroyAllWindows()
