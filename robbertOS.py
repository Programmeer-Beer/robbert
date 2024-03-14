######## Count Change Using Object Detection #########
#
# Author: Evan Juras, EJ Technology Consultants (www.ejtech.io)
# Date: 10/29/22
#  
# Description:
# This program uses a TFLite coin detection model to locate and identify coins in 
# a live camera feed. It calculates the total value of the coins in the camera's view.
# (Works on US currency, but can be modified to work with coins from other countries!)

# Import packages
import os
import argparse
import cv2
import numpy as np
import sys
import time
from threading import Thread
import importlib.util

import rospy as rp
from geometry_msgs.msg import Pose, Vector3
from mirte_msgs.msg import *
from mirte_msgs.srv import *
from sensor_msgs.msg import *
from std_srvs.srv import *


SetRightSpeed = rp.ServiceProxy('/mirte/set_right_speed', SetMotorSpeed)
SetLeftSpeed = rp.ServiceProxy('/mirte/set_left_speed', SetMotorSpeed)
GetDistanceLeft = rp.ServiceProxy('/mirte/get_distance_left',GetDistance)
GetPinValue = rp.ServiceProxy('/mirte/get_pin_value', GetPinValue)
SetLeftServo=rp.ServiceProxy('/mirte/set_left_servo_angle',SetServoAngle)

### User-defined variables

# Model info
# MODEL_NAME = 'aluminium_papier_model'
# GRAPH_NAME = 'detect.tflite'
# LABELMAP_NAME = 'labelmap.txt'
# use_TPU = False
# Define and parse input arguments
parser = argparse.ArgumentParser()
#parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
#                    required=True)
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='640x480')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

#MODEL_NAME = args.modeldir
MODEL_NAME = 'aluminium_papier_model'
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu


# Program settings
min_conf_threshold = 0.50
resW, resH = 640, 480 # Resolution to run camera at
imW, imH = resW, resH

###setup grabber
Holding_item=False

### Set up model parameters

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
ret = cap.set(3, resW)
ret = cap.set(4, resH)

# Initialize frame rate calculation
frame_rate_calc = 1
freq = cv2.getTickFrequency()

### Continuously process frames from camera
while True:

    # Start timer (for calculating frame rate)
    t1 = cv2.getTickCount()

    # Reset coin value count for this frame
    number_of_objects = 0

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

    #defining loc dict 
    loc_closest= {
        'width': 0,
        'xcenter' :0,
        'ycenter' :0,
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

            # Draw bounding box
            cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

            # Get object's name and draw label
            object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
            label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'quarter: 72%'
            labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
            cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
            cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

            # Assign the value of this coin based on the class name of the detected object
            # (There are more efficient ways to do this, but this shows an example of how to trigger an action when a certain class is detected)
            if object_name == 'aluminium':
                object_material = 'aluminium'
            elif object_name == 'papier':
                object_material = 'papier'

            object_width = xmax - xmin
            object_distance = 77.48 - 0.29 * object_width
            number_of_objects = number_of_objects + 1

                # Now that we've gone through every detection, we know the total value of all coins in the frame. Let's display it in the corner of the frame.
            #cv2.putText(frame,'Object gevonden: ' + object_material,(20,40+40*number_of_objects),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),4,cv2.LINE_AA)
            cv2.putText(frame,object_material + ' gevonden op ' + '%.2f' % object_distance + ' cm afstand',(20,40 + 20*number_of_objects),cv2.FONT_HERSHEY_PLAIN,1,(230,230,230),2,cv2.LINE_AA)
            
            #creation of coordinated, size and material dict
            if (xmax-xmin)>loc_closest['width'] and Holding_item==False:
                loc_closest= {
                    'width': abs(xmax-xmin),
                    'xcenter' :(xmax-xmin)/2,
                    'ycenter' :(ymax-ymin)/2,
                    'material': object_material,
            }
                
    # Now that we've gone through every detection, we know the total value of all coins in the frame. Let's display it in the corner of the frame.
    # cv2.putText(frame,'Total change:',(20,80),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),4,cv2.LINE_AA)
    # cv2.putText(frame,'Total change:',(20,80),cv2.FONT_HERSHEY_PLAIN,2,(230,230,230),2,cv2.LINE_AA)
    # cv2.putText(frame,'$%.2f' % total_coin_value,(260,85),cv2.FONT_HERSHEY_PLAIN,2.5,(0,0,0),4,cv2.LINE_AA)
    # cv2.putText(frame,'$%.2f' % total_coin_value,(260,85),cv2.FONT_HERSHEY_PLAIN,2.5,(85,195,105),2,cv2.LINE_AA)

    # Draw framerate in corner of frame
    cv2.putText(frame,'FPS: %.2f' % frame_rate_calc,(20,50),cv2.FONT_HERSHEY_PLAIN,2,(0,0,0),4,cv2.LINE_AA)
    cv2.putText(frame,'FPS: %.2f' % frame_rate_calc,(20,50),cv2.FONT_HERSHEY_PLAIN,2,(230,230,230),2,cv2.LINE_AA)

    # All the results have been drawn on the frame, so it's time to display it.
    cv2.imshow('Object detector', frame)

    # Calculate framerate
    t2 = cv2.getTickCount()
    time1 = (t2-t1)/freq
    frame_rate_calc= 1/time1


    
    
    # if size<size_new nothin else update size and loc
    #movement code
    loc_x= int((loc_closest['xcenter']))
    loc_y= int((loc_closest['ycenter']))
    #coordinate=[x,y]


    x_reference=resW/2
    kx=1 #to edit
    x_deviation=loc_x-x_reference
    #if x is to the left of x_refrence i.e x<x_refrence
    # x_error<0 so the left motor rotates backwards and the right motor rotates forward
    # this rotates the robot to the left
    while 3>x_deviation>-3: #margine TBT.
        s= kx*x_deviation
        SetLeftSpeed(s)
        SetRightSpeed(-s) 
    


    y_reference=resH/4 #to edit
    ky=1 #to edit
    y_deviation=loc_y-y_reference
    while 0<y_deviation<3:
        u= ky*y_deviation
        SetLeftSpeed(u)
        SetRightSpeed(u) 

    # Press 'q' to quit
    if cv2.waitKey(1) == ord('q'):
        break

# Clean up
cv2.destroyAllWindows()
cap.release()


