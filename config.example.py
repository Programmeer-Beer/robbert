# Environment
ENV = "production" # development or production
DEBUG = True
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
MIN_DISTANCE = 30  # [cm]
AVOID_COLLISSION_SPEED = 40  # [%]
AVOID_COLLISSION_BACKWARD_DURATION = 0.5  # [s]
AVOID_COLLISSION_TURN_DURATION = 0.5  # [s]
TURN_DURATION = 0.1  # [s]
DROPPED_TRASH_SPEED = 20  # [s]
DROPPED_TRASH_DURATION = 1  # [s]
NON_GRABBED_SERVO_ANGLE= 160  # [degrees]
GRABBED_SERVO_ANGLE = 20  # [degrees]
LEFT_DELAY = 0.0  # [s]
MAX_REST = 0.4  # [s]
LEFT_SPEED = 100  # [%]
RIGHT_SPEED = 90  # [%]
MINIMUM_WHITE = 2000  # geen idee

TURN_LEFT = {
    'direction': 'left',
    'speed': AVOID_COLLISSION_SPEED,
    'duration': TURN_DURATION,
}
TURN_LEFT_AVOID_COLLISSION = {
    'direction': 'left',
    'speed': AVOID_COLLISSION_SPEED,
    'duration': AVOID_COLLISSION_TURN_DURATION,
}
BACKWARD = {
    'direction': 'backward',
    'speed': DROPPED_TRASH_SPEED,
    'duration': DROPPED_TRASH_DURATION,
}
LEFT_MOTOR_DEVIATION = 1 # Multiplycation factor
RIGHT_MOTOR_DEVIATION = 1 # Multiplycation factor

# Object detection
MODEL_NAME = "aluminium_papier_model_v2"
SHOW_FRAME = False
resW = 640  # [px]
resH = 480  # [px]
min_conf_threshold = 0.80  # [%]

# Calculate
DEVIATION = 10  # [%]
DISERED_OBJECT_LOCATION = 10  # [%]
CAUTION_LEVEL_X = 0.002  # CAUTION_LEVEL = duration[s] / deviation[%]
CAUTION_LEVEL_Y = 0.005  # CAUTION_LEVEL = duration[s] / deviation[%]
STANDARD_SPEED = 90  # [%]
STANDARD_DURATION_SONAR_FORWARD = 0.1 # [s]
STANDARD_DURATION_SONAR_TURNING = 0.1 # [s]
DROP_DISTANCE = 25  # [cm]