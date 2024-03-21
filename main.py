##########################################################
##  _____            _       _                     _    ##
## |  __ \          | |     | |                   | |   ##
## | |__) |   ___   | |__   | |__     ___   _ __  | |_  ##
## |  _  /   / _ \  | '_ \  | '_ \   / _ \ | '__| | __| ##
## | | \ \  | (_) | | |_) | | |_) | |  __/ | |    | |_  ##
## |_|  \_\  \___/  |_.__/  |_.__/   \___| |_|     \__| ##
##########################################################
##               Hoe robbertOS gebruiken                ##
##########################################################
#  Vind het dichtstbij zijnde object
#  object_closest['xpercentage'], object_closest['ypercentage'], object_closest['material'] = object_detection.closest()
#    return[0] = object_closest['xpercentage']: 0 <= xpercentage <= 100 [% vanaf links]
#    return[1] = object_closest['ypercentage']: 0 <= ypercentage <= 100 [% vanaf onder]
#    return[2] = object_closest['material']: 'aluminium' || 'papier'
# 
#  Robbert laten bewegen
#  robbert.move(direction, speed, duration)
#    directions: 'forward' || 'backward' || 'right' || 'left'
#    speeds: 0 <= speed <= 100 [%]
#    duration: 0 <= duration [s]
# 
#  Robbert laten oppakken
#  robbert.grab()
# 
#  Robbert laten checken op potentiële botsing
#  robbert.collsion_warning()
#    return = warnings: True || False
# 
#  Ontwijken berekenen
#  calculate.search_movement(object_closest)
#    return[0] = directions: 'forward' || 'backward' || 'right' || 'left'
#    return[1] = speeds: 0 <= speed <= 100 [%]
#    return[2] = duration: 0 <= duration [s]
#    return[3] = grab: True || False
##########################################################
##                                                      ##
##########################################################

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