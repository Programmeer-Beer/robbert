from config import ENV, DEBUG, DEVIATION, DISERED_OBJECT_LOCATION, CAUTION_LEVEL_X, CAUTION_LEVEL_Y, STANDARD_SPEED, DROP_DISTANCE, STANDARD_DURATION_SONAR_FORWARD, STANDARD_DURATION_SONAR_TURNING

min_deviation = 50 - DEVIATION
max_deviation = 50 + DEVIATION

def search_movement(object_closest):
	advised_movement = {
        'direction': 'forward',
        'speed': STANDARD_SPEED,
        'duration': 0,
        'grab': False,
    }

    if DEBUG:
	 print(object_closest)
	
	if min_deviation <= object_closest['xpercentage'] <= max_deviation:
		# Object is on the centerline
		print('Op de centerline')
		if object_closest['ypercentage'] <= DISERED_OBJECT_LOCATION:
			# Object is in the position to be grabbed
			advised_movement['grab'] = True
		else:
			advised_movement['direction'] = 'forward'
			deviation = object_closest['ypercentage']
	else:
		if object_closest['xpercentage'] < min_deviation:
			# Object is to the left of the centerline
			advised_movement['direction'] = 'left'
			deviation = 50 - object_closest['xpercentage']
		elif object_closest['xpercentage'] > max_deviation:
			# Object is to the right of the centerline
			advised_movement['direction'] = 'right'
			deviation = object_closest['xpercentage'] - 50
		advised_movement['duration'] = duration(deviation, advised_movement['direction'])

		if DEBUG:
			print('duration :' + str(advised_movement['duration']))
			print(duration(deviation, advised_movement['direction']))
	
	return advised_movement

def drop_movement(color, desired_color, distance):
	advised_movement = {
        'direction': 0,
        'speed': STANDARD_SPEED,
        'duration': 0,
        'drop': False,
    }

	if color['left'] == color['right']:
		if color['left'] == desired_color and distance <= DROP_DISTANCE:
			# Als beide sensoren hetzelfde binnen krijgen en de kleur is goed drop
			advised_movement['drop'] = True
		else:
			# Als alleen beide sensoren hetzelfde binnen krijgen, rijdt vooruit.
			# Als hij dreigt te botsen wijkt hij namelijk al uit.
			advised_movement['direction'] = 'forward'
			advised_movement['duration'] = STANDARD_DURATION_SONAR_FORWARD
	else:
		# Draaien
		advised_movement['duration'] = STANDARD_DURATION_SONAR_TURNING
		if color['left'] == desired_color:
			advised_movement['direction'] = 'left'
		else:
			advised_movement['direction'] = 'right'

	return advised_movement

def duration(deviation, direction):
	# Om er achter te komen hoe snel Robbert kan bewegen deze berekening
	# De devition is altijd tussen de 0 en 50
	# We willen dat als hij 50 is, hij het langst beweegt
	# We willen dat als hij 0 is, hij het kortst
	#
	# duration = 50 [deviation] * [CAUTION_LEVEL]
	#
	# Als wij een kwart seconde willen voor het langste kunnen wij dat berekenen door
	# CAUTION_LEVEL = duration[s] / deviation[%]
	# CAUTION_LEVEL = 0.25 / 50 = 0.005 

	if direction == 'forward':
		duration = deviation * CAUTION_LEVEL_Y + 0.1
	else:
		duration = deviation * CAUTION_LEVEL_X + 0.1

	return duration