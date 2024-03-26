from config import ROBBERT_LOGO, TURN_LEFT, BACKWARD
import os
import warnings
import calculate
import object_detection
import robbert

def welcome_message():
    if os.name == 'nt':  # nt betekent Windows
        os.system('cls')  # Clear commando voor Windows
    else:
        os.system('clear')  # Clear commando voor Unix/Linux/macOS
    
    print(ROBBERT_LOGO)

def search_object():
    object_material = ''
    while True:
        if robbert.collision_warning() == True:
            # If object detected: revert, turn left and the loop starts over
            robbert.avoid_collision()
        else:
            # Search closest object
            object_closest = object_detection.find_closest()
            if object_closest['material'] == '':
                # If no object is found, make a left turn and the loop starts over
                print('Geen afval gevonden, ik zoek door.')
                robbert.move(TURN_LEFT)
            else:
                # If object is found, calculate the movement necessary to reach the object
                advised_movement = calculate.search_movement(object_closest)
                if advised_movement['grab']:
                    # If advised by the calculator, grab the object. This ends the while loop
                    object_material = object_closest['material']
                    break
                print(advised_movement)
                robbert.move(advised_movement)

    # Grab the object
    print('Ja! Ik pak dit propje ' + object_material)
    robbert.grab()

    return object_material

def drive_to_drop(object_material):
    if object_material == 'aluminium':
        desired_color = 'white'
    else:
        desired_color = 'black'

    print('Nu ga ik dit stukje ' + object_material + ' wegbrengen naar ' + desired_color + '!')
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

    print(object_material + ' gedropt op ' + desired_color + '.')
    robbert.drop()

def clean_trash():
    while True:
        object_material = search_object()
        drive_to_drop(object_material)
        robbert.move(BACKWARD)
        sleep(10)

def main():
    welcome_message()
    clean_trash()

if __name__ == "__main__":
    main()