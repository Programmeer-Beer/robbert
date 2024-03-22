from config import ENV, MODEL_NAME, resW, resH, min_conf_threshold

if ENV == 'production':
	import os
	import argparse
	import cv2
	import numpy as np
	import sys
	import multiprocessing
	import time
	from threading import Thread
	import importlib.util

	parser = argparse.ArgumentParser()
	parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
	                    default='detect.tflite')
	parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
	                    default='labelmap.txt')
	parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
	                    action='store_true')

	args = parser.parse_args()
	GRAPH_NAME = args.graph
	LABELMAP_NAME = args.labels
	imW, imH = int(resW), int(resH)
	use_TPU = args.edgetpu

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

	### Load Tensorflow Lite model
	# If using Edge TPU, use special load_delegate argument
	if use_TPU:
	    interpreter = Interpreter(model_path=PATH_TO_CKPT,
	                              experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
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

	# Initialize camera
	cap = cv2.VideoCapture(0)
	ret = cap.set(3, imW)
	ret = cap.set(4, imH)


	### Continuously process frames from camera
	def find_closest():
	    # Grab frame from camera
	    hasFrame, frame1 = cap.read()

	    # Acquire frame and resize to input shape expected by model [1xHxWx3]
	    frame = frame1.copy()
	    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
	    frame_resized = cv2.resize(frame_rgb, (width, height))
	    input_data = np.expand_dims(frame_resized, axis=0)

	    # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
	    if floating_model:
	        input_data = (np.float32(input_data) - input_mean) / input_std

	    # Perform detection by running the model with the image as input
	    interpreter.set_tensor(input_details[0]['index'],input_data)
	    interpreter.invoke()

	    # Retrieve detection results
	    boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
	    classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
	    scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects

	    object_closest = {
	        'xpercentage': 0,
	        'ypercentage': 0,
	        'material': '',
	    }

	    # Loop over all detections and process each detection if its confidence is above minimum threshold
	    for i in range(len(scores)):
	        if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

	            # Get bounding box coordinates
	            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
	            ymin = int(max(1,(boxes[i][0] * imH)))
	            xmin = int(max(1,(boxes[i][1] * imW)))
	            ymax = int(min(imH,(boxes[i][2] * imH)))
	            xmax = int(min(imW,(boxes[i][3] * imW)))
	            xcenter = (xmax + xmin) / 2
	            ycenter = resH - (ymax + ymin) / 2
	            xpercentage= round((xcenter / resW) * 100)
	            ypercentage= round((ycenter / resH) * 100)
	            # Draw bounding box
	            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

	            # Get object's name and draw label
	            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
	            label = '%s: %d%%: %s: %s' % (object_name, int(scores[i]*100), xpercentage, ypercentage) # Example: 'quarter: 72%'
	            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
	            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
	            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
	            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

	            object_width = xmax - xmin
	            object_distance = 77.48 - 0.29 * object_width
	            number_of_objects = number_of_objects + 1

	            # Now that we've gone through every detection, we know the total value of all coins in the frame. Let's display it in the corner of the frame.
	            cv2.putText(frame,object_name + ' gevonden op ' + '%.2f' % object_distance + ' cm afstand',(20,40 + 20*number_of_objects),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),4,cv2.LINE_AA)
	            cv2.putText(frame,object_name + ' gevonden op ' + '%.2f' % object_distance + ' cm afstand',(20,40 + 20*number_of_objects),cv2.FONT_HERSHEY_PLAIN,1,(230,230,230),2,cv2.LINE_AA)


	            if ypercentage < object_closest['ypercentage']:
	                object_closest = {
	                    'xpercentage': xpercentage,
	                    'ypercentage': ypercentage,
	                    'material': object_name,
	                }

	    # All the results have been drawn on the frame, so it's time to display it.
	    cv2.imshow('Object detector', frame)

	    return object_closest
else:
	def find_closest():
		object_closest = {
			'xpercentage': 50,
			'ypercentage': 8,
			'material': 'papier',
		}

		return object_closest