HOE ROBBERT GEBRUIKEN?
Vind het dichtstbij zijnde object
object_closest['xpercentage'], object_closest['ypercentage'], object_closest['material'] = object_detection.closest()
return[0] = object_closest['xpercentage']: 0 <= xpercentage <= 100 [% vanaf links]
return[1] = object_closest['ypercentage']: 0 <= ypercentage <= 100 [% vanaf onder]
return[2] = object_closest['material']: 'aluminium' || 'papier'

Robbert laten bewegen
robbert.move(direction, speed, duration)
directions: 'forward' || 'backward' || 'right' || 'left'
speeds: 0 <= speed <= 100 [%]
duration: 0 <= duration [s]

Robbert laten oppakken
robbert.grab()

Robbert laten checken op potentiële botsing
robbert.collsion_warning()
return = warnings: True || False

Ontwijken berekenen
calculate.search_movement(object_closest)
return[0] = directions: 'forward' || 'backward' || 'right' || 'left'
return[1] = speeds: 0 <= speed <= 100 [%]
return[2] = duration: 0 <= duration [s]
return[3] = grab: True || False
