from config import ROBBERT_LOGO, TURN_LEFT, FORWARD, BACKWARD, NUMBER_OF_FIRST_CIRCLES, NUMBER_OF_SECOND_CIRCLES, NUMBER_OF_FORWARDS
import os
import warnings
import calculate
import object_detection
import robbert
import time

def welcome_message():
    if os.name == 'nt':  # nt betekent Windows
        os.system('cls')  # Clear commando voor Windows
    else:
        os.system('clear')  # Clear commando voor Unix/Linux/macOS
    
    print(ROBBERT_LOGO)

def search_object():
    print('Ik ga nieuw afval zoeken.')
    object_material = ''
    i = 0
    j = 0
    k = 0
    
    while True:
        if robbert.collision_warning() == True:
            # If object detected: revert, turn left and the loop starts over
            robbert.avoid_collision()
        else:
            # Search closest object
            time.sleep(0.1)
            object_closest = object_detection.find_closest()
            if object_closest['material'] == '':
                # If he sees nothing for two circles: move forward, make a circle and move forward and repeat
                if i > NUMBER_OF_FIRST_CIRCLES and j >= NUMBER_OF_SECOND_CIRCLES and k < NUMBER_OF_FORWARDS:
                    robbert.move(FORWARD)
                    if k == NUMBER_OF_FORWARDS:
                        k = 0
                        j = 0
                    else:
                        k += 1
                # If no object is found, make a left turn and the loop starts over
                robbert.move(TURN_LEFT)
                i += 1
                j += 1
            else:
                i = 0
                j = 0
                # If object is found, calculate the movement necessary to reach the object
                print('Afval gevonden!')
                advised_movement = calculate.search_movement(object_closest)
                if advised_movement['grab']:
                    # If advised by the calculator, grab the object. This ends the while loop
                    robbert.move(FORWARD)
                    object_material = object_closest['material']
                    break
                robbert.move(advised_movement)

    # Grab the object
    print('Ik pak het ' + object_material + ' propje op.')
    robbert.grab()

    return object_material

def drive_to_drop(object_material):
    if object_material == 'aluminium':
        desired_color = 'white'
    else:
        desired_color = 'black'

    print('Ik breng het ' + object_material + ' propje naar ' + desired_color + '.')
    while True:
        if robbert.collision_warning() == True:
            # If object detected: revert, turn left and the loop starts over
            robbert.avoid_collision()
        else:
            color = robbert.check_color()  # color['left'], color['right']
            distance = robbert.sonar_distance()  # [m]
            advised_movement = calculate.drop_movement(color, desired_color, distance)
            if advised_movement['drop']:
                # If advised by the calculator, drop the object. This ends the while loop
                break
            robbert.move(advised_movement)

    # Drop the object
    print(object_material + ' gedropt op ' + desired_color + '.')
    robbert.drop()

def clean_trash():
    while True:
        object_material = search_object()
        drive_to_drop(object_material)
        robbert.move(BACKWARD)
        time.sleep(10)

def main():
    welcome_message()
    time.sleep(15)  # Gives time to put Robbert down
    clean_trash()

if __name__ == "__main__":
    main()