########################################################
###                   Enviroment                     ###
########################################################
ENV = "production" # development or production
DEBUG = True
SHOW_FRAME = False
ROBBERT_LOGO = """
  _____            _       _                     _   
 |  __ \          | |     | |                   | |  
 | |__) |   ___   | |__   | |__     ___   _ __  | |_ 
 |  _  /   / _ \  | '_ \  | '_ \   / _ \ | '__| | __|
 | | \ \  | (_) | | |_) | | |_) | |  __/ | |    | |_ 
 |_|  \_\  \___/  |_.__/  |_.__/   \___| |_|     \__|
                                                     
                                                     
"""

NUMBER_OF_FIRST_CIRCLES = 30
NUMBER_OF_SECOND_CIRCLES = 20
NUMBER_OF_FORWARDS = 10

########################################################
###                   Calculate                      ###
########################################################
# Driving constants
STANDARD_SPEED = 100  # [%]

# Location constants
DEVIATION = 20  # [%]
CAMERA_DEVIATION_TO_RIGHT = 0  # [%]
DISERED_OBJECT_LOCATION = 12  # [%]
DROP_DISTANCE = 40  # [cm]

# Duration constants / The lower, the shorter
CAUTION_LEVEL_X = 0.001  # CAUTION_LEVEL = duration[s] / deviation[%]
CAUTION_LEVEL_Y = 0.005  # CAUTION_LEVEL = duration[s] / deviation[%]

# Driving constants
STANDARD_DURATION_SONAR_FORWARD = 0.5 # [s]
STANDARD_DURATION_SONAR_TURNING = 0.1 # [s]



########################################################
###                    Robbert                       ###
########################################################
# Servo configuration
STALL_PERCENTAGE = 40  # [%]
LEFT_DELAY = 0.0  # [s]
RIGHT_DELAY = 0.0  # [s]
MAX_REST = 0.4  # [s]
LEFT_SPEED = 100  # [%]
RIGHT_SPEED = 90  # [%]
LEFT_MOTOR_DEVIATION = 1 # Multiplycation factor
RIGHT_MOTOR_DEVIATION = 1 # Multiplycation factor

# Grabber constants
NON_GRABBED_SERVO_ANGLE= 160  # [degrees]
GRABBED_SERVO_ANGLE = 20  # [degrees]

# Intensity constants
MINIMUM_WHITE = 2000  # geen idee

# Avoid object constants
MIN_DISTANCE = 30  # [cm]
AVOID_COLLISSION_SPEED = 40  # [%]
AVOID_COLLISSION_BACKWARD_DURATION = 0.5  # [s]
AVOID_COLLISSION_TURN_DURATION = 0.5  # [s]
TURN_LEFT_AVOID_COLLISSION = {
    'direction': 'left',
    'speed': AVOID_COLLISSION_SPEED,
    'duration': AVOID_COLLISSION_TURN_DURATION,
}

# General driving standarts
NORMAL_DURATION = 0.3  # [s]
FORWARD = {
    'direction': 'forward',
    'speed': STANDARD_SPEED,
    'duration': NORMAL_DURATION,
}
TURN_DURATION = 0.05  # [s]
TURN_LEFT = {
    'direction': 'left',
    'speed': STANDARD_SPEED,
    'duration': TURN_DURATION,
}
DROPPED_TRASH_DURATION = 1  # [s]
BACKWARD = {
    'direction': 'backward',
    'speed': STANDARD_SPEED,
    'duration': DROPPED_TRASH_DURATION,
}



########################################################
###               Object Detection                   ###
########################################################
# Model specifications
MODEL_NAME = "aluminium_papier_model_v2"
min_conf_threshold = 0.95  # [%]

# Camera specifications
resW = 640  # [px]
resH = 480  # [px]