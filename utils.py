def resizeCoords(data, oldSize, newSize):
	ratio = newSize / oldSize
	goals = data['map']['goals']
	obstacles = data['map']['obstacles']
	path = data['path']

	for goal in goals:
		goal['width'] *= ratio
		goal['height'] *= ratio
		goal['location']['x'] *= ratio
		goal['location']['y'] *= ratio
		goal['location']['z'] *= ratio

	for obstacle in obstacles:
		obstacle['width'] *= ratio
		obstacle['height'] *= ratio
		obstacle['location']['x'] *= ratio
		obstacle['location']['y'] *= ratio
		obstacle['location']['z'] *= ratio

	for state in path:
		for id, uav in state.items():
			uav['pointParticle']['location']['x'] *= ratio
			uav['pointParticle']['location']['y'] *= ratio
			uav['pointParticle']['location']['z'] *= ratio

	return data



def moveCoords(data, offsetX, offsetY, offsetZ):
	goals = data['map']['goals']
	obstacles = data['map']['obstacles']
	path = data['path']

	for goal in goals:
		goal['location']['x'] += offsetX
		goal['location']['y'] += offsetY
		goal['location']['z'] += offsetZ

	for obstacle in obstacles:
		obstacle['location']['x'] += offsetX
		obstacle['location']['y'] += offsetY
		obstacle['location']['z'] += offsetZ

	for state in path:
		for id, uav in state.items():
			uav['pointParticle']['location']['x'] += offsetX
			uav['pointParticle']['location']['y'] += offsetY
			uav['pointParticle']['location']['z'] += offsetZ

	return data


def moveCoordsToObjectMiddle(data):
	goals = data['map']['goals']
	obstacles = data['map']['obstacles']
	path = data['path']

	for goal in goals:
		goal['location']['x'] += goal['width'] / 2
		goal['location']['y'] += goal['height'] / 2

	for obstacle in obstacles:
		obstacle['location']['x'] += obstacle['width'] / 2
		obstacle['location']['y'] += obstacle['height'] / 2

	return data

