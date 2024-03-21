from config import ROBBERT_LOGO
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
            # Als er een object of muur in de weg is rij naar achter, dan naar rechts en begin de loop opnieuw
            robbert.avoid_collision()
        else:
            # Zoek het dichtstbij zijnde object
            object_closest = object_detection.find_closest()
            if object_closest['material'] == '':
                # Als er geen object is gevonden, roteer dan en begin de loop opnieuw
                print('Geen afval gevonden, ik zoek door.')
                robbert.move('left', 20, 0.1)
            else:
                # Als er een object is gevonden, bereken hoe je ernaar toe moet
                advised_movement = calculate.search_movement(object_closest)
                if advised_movement['grab']:
                    # Als wordt geadviseerd het object te pakken, beëindig de while-loop
                    object_material = object_closest['material']
                    break
                print(advised_movement)
                robbert.move(advised_movement)

    # Pak het object
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
            # Als er een object of muur in de weg is rij naar achter, dan naar rechts en begin de loop opnieuw
            print('AAAAAAAAAAHH!! IK GA BOTSEN!!! IK DRAAI OM HOOR!')
            robbert.avoid_collision()
        else:
            color = robbert.check_color()  # color['left'], color['right']
            distance = robbert.sonar_distance()  # [m]
            advised_movement = calculate.drop_movement(color, desired_color, distance)
            if advised_movement['drop']:
                # Als wordt geadviseerd het object te droppen, beëindig de while-loop
                break
            robbert.move(advised_movement)

    print(object_material + ' gedropt op ' + desired_color + '.')
    robbert.drop()

def clean_trash():
    object_material = search_object()
    drive_to_drop(object_material)
    print('Robbert out')

def main():
    welcome_message()
    clean_trash()

if __name__ == "__main__":
    main()