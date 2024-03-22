from config import ENV, STALL_PERCENTAGE, MIN_DISTANCE, AVOID_COLLISSION_SPEED, AVOID_COLLISSION_BACKWARD_DURATION, AVOID_COLLISSION_TURN_DURATION, TURN_LEFT
import time

if ENV == 'production':
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
	SetLeftServo = rp.ServiceProxy('/mirte/set_left_servo_angle',SetServoAngle)

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
    absolute_speed = STALL_PERCENTAGE + speed * active_percentage

    # Combine motor speed with direction  
    left_speed =  absolute_speed * left_direction
    right_speed = absolute_speed * right_direction

    # Push to motor
    if ENV == 'production':
        SetLeftSpeed(int(left_speed))
        SetRightSpeed(int(right_speed))
    else:
        print('Left speed: ' + str(left_speed) + ' real / ' + str(speed * left_direction) + ' desired')
        print('Right speed: ' + str(right_speed) + ' real / ' + str(speed * right_direction) + ' desired')
        print('Execution time: ' + str(duration))

    # Determines how long Robbert will have its motors on
    time.sleep(duration)

    # Turn motors off
    if ENV == 'production':
        SetLeftSpeed(0)
        SetRightSpeed(0)
    else:
        print('Motors turned off')

def collision_warning():
	# Checks if objects are too close and returns True or False

    distance = sonar_distance()  # [m]

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
    move(TURN_LEFT)

def grab():
    # Dit had je staan bij setup grabber
    # Holding_item=False
    # x_reference=resW/2
    # y_reference=resH/4

    if ENV == 'production':
        print('You forgot to write the grabbing code :(')
    else:
        print('Opgepakt!')

def drop():
    #drop object
    if ENV == 'production':
        print('You forgot to write the dropping code :(')
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
    if ENV == 'production':
        # Read color sensor en geef aan of het zwart of wit is
        color = {
            'left': '',
            'right': '',
        }
    else:
        color = {
            'left': 'black',
            'right': 'black',
        }
    
    return color