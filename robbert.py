from config import ENV, DEBUG, MINIMUM_WHITE, MAX_REST, LEFT_DELAY, LEFT_SPEED, RIGHT_SPEED, STALL_PERCENTAGE, MIN_DISTANCE, AVOID_COLLISSION_SPEED, AVOID_COLLISSION_BACKWARD_DURATION, AVOID_COLLISSION_TURN_DURATION, TURN_LEFT_AVOID_COLLISSION, TURN_LEFT, LEFT_MOTOR_DEVIATION, RIGHT_MOTOR_DEVIATION, NON_GRABBED_SERVO_ANGLE, GRABBED_SERVO_ANGLE
import time

if ENV == 'production':
    import rospy as rp
    from geometry_msgs.msg import Pose, Vector3
    from mirte_msgs.msg import *
    from mirte_msgs.srv import *
    from sensor_msgs.msg import *
    from std_srvs.srv import *
    from mirte_robot import robot
    mirte = robot.createRobot()

    SetRightSpeed = rp.ServiceProxy('/mirte/set_right_speed', SetMotorSpeed)
    SetLeftSpeed = rp.ServiceProxy('/mirte/set_left_speed', SetMotorSpeed)
    GetDistanceLeft = rp.ServiceProxy('/mirte/get_distance_left',GetDistance)
    GetPinValue = rp.ServiceProxy('/mirte/get_pin_value', GetPinValue)
    SetLeftServo = rp.ServiceProxy('/mirte/set_left_servo_angle',SetServoAngle)

    # Reset grabber angle to starting posistion
    SetLeftServo(NON_GRABBED_SERVO_ANGLE)

# Calculate active the amount off power possible so that the imput can be 0% to 100%
active_percentage = (100 - STALL_PERCENTAGE) / 100

def move(advised_movement):
    direction = advised_movement['direction']
    speed = advised_movement['speed']
    duration = advised_movement['duration']
    
    # Set direction
    if direction == 'forward':
        left_direction, right_direction = 1, 1
    elif direction == 'backward':
        left_direction, right_direction = -1, -1
    elif direction == 'left':
        left_direction, right_direction = -1, 1
    elif direction == 'right':
        left_direction, right_direction = 1, -1

   	# Set motor speed
    # absolute_speed = STALL_PERCENTAGE + speed * active_percentage

    # Combine motor speed with direction  
    # left_speed = absolute_speed * left_direction * LEFT_MOTOR_DEVIATION
    # right_speed = absolute_speed * right_direction * RIGHT_MOTOR_DEVIATION
    left_speed = LEFT_SPEED * left_direction
    right_speed = RIGHT_SPEED * right_direction

    # rest = LEFT_DELAY * duration
    # if rest > MAX_REST:
    #     rest = MAX_REST
    
    # Push to motor
    if ENV == 'production':
        SetLeftSpeed(int(left_speed))
        # time.sleep(rest)
        time.sleep(0.1)
        SetRightSpeed(int(right_speed))
    else:
        print('Left speed: ' + str(left_speed) + ' real / ' + str(speed * left_direction) + ' desired')
        print('Right speed: ' + str(right_speed) + ' real / ' + str(speed * right_direction) + ' desired')
        print('Execution time: ' + str(duration))

    # Determines how long Robbert will have its motors on
    time.sleep(duration)

    # Turn motors off
    if ENV == 'production':
        SetRightSpeed(0)
        time.sleep(0.05)
        SetLeftSpeed(0)
    else:
        print('Motors turned off')

def collision_warning():
	# Checks if objects are too close and returns True or False

    distance = sonar_distance() # [cm]
    
    if DEBUG:
        print('Sonar distance: ' + str(distance))

    if distance < MIN_DISTANCE:
        collision_warning = True
    else:
        collision_warning = False

    return collision_warning

def avoid_collision():
    print('AAAAAAAAAAHH!! IK GA BOTSEN!!! IK DRAAI OM HOOR!')
    advised_movement = {
        'direction': 'backward',
        'speed': AVOID_COLLISSION_SPEED,
        'duration': AVOID_COLLISSION_BACKWARD_DURATION,
    }
    move(advised_movement)
    move(TURN_LEFT_AVOID_COLLISSION)

def grab():
    # Grab object
    if ENV == 'production':
        SetLeftServo(GRABBED_SERVO_ANGLE)
    else:
        print('Opgepakt!')

def drop():
    # Drop object
    if ENV == 'production':
        SetLeftServo(NON_GRABBED_SERVO_ANGLE)
    else:
        print('Gedropt!')

def sonar_distance():
    if ENV == 'production':
        distance_raw = GetDistanceLeft()
        distance = int(distance_raw.data)
    else:
        distance = 0.15 # Read sonar

    return distance

def check_color(): # color['left'], color['right']
    # Read color sensor and return colors
    color = {
        'left': 'black',
        'right': 'black',
    }

    if ENV == 'production':
        if DEBUG:
            print(mirte.getIntensity('left'))
            print(mirte.getIntensity('right'))
        intensity_left_raw = mirte.getIntensity('left')
        intensity_right_raw = mirte.getIntensity('right')
        intensity_left = int(intensity_left_raw)
        intensity_right = int(intensity_right_raw)
        if intensity_left > MINIMUM_WHITE:
            color['left'] = 'white'
        if intensity_right > MINIMUM_WHITE:
            color['right'] = 'white'
    
    return color