# Environment
ENV = "production" # development or production
ROBBERT_LOGO = """
  _____            _       _                     _   
 |  __ \          | |     | |                   | |  
 | |__) |   ___   | |__   | |__     ___   _ __  | |_ 
 |  _  /   / _ \  | '_ \  | '_ \   / _ \ | '__| | __|
 | | \ \  | (_) | | |_) | | |_) | |  __/ | |    | |_ 
 |_|  \_\  \___/  |_.__/  |_.__/   \___| |_|     \__|
                                                     
                                                     
"""

# Robbert
STALL_PERCENTAGE = 40  # [%]
MIN_DISTANCE = 0.1  # [m]
AVOID_COLLISSION_SPEED = 20  # [%]
AVOID_COLLISSION_BACKWARD_DURATION = 0.5  # [s]
AVOID_COLLISSION_TURN_DURATION = 0.1  # [s]

# Object detection
MODEL_NAME = "aluminium_papier_model"
GRAPH_NAME = "detect.tflite"
LABELMAP_NAME = "labelmap.txt"
imW = 640  # [px]
imH = 480  # [px]
resW = 640  # [px]
resH = 480  # [px]
use_TPU = "store_true"
min_conf_threshold = 0.80  # [%]

# Calculate
DEVIATION = 10  # [%]
DISERED_OBJECT_LOCATION = 10  # [%]
CAUTION_LEVEL_X = 0.005  # CAUTION_LEVEL = duration[s] / deviation[%]
CAUTION_LEVEL_Y = 0.005  # CAUTION_LEVEL = duration[s] / deviation[%]
STANDARD_SPEED = 10  # [%]
STANDARD_DURATION_SONAR_FORWARD = 0.1 # [s]
STANDARD_DURATION_SONAR_TURNING = 0.1 # [s]
DROP_DISTANCE = 0.2  # [m]