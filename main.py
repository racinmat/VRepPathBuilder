import vrep
import sys
import json
import pprint
import time

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


print ('Program started')

# načtení cesty

with open('path.json') as data_file:
    data = json.load(data_file)

# pprint.pprint(data)

data = resizeCoords(data, 1000, 5)	# 1000 je velikost mapy při simulaci, 5 je velikost mapy ve vrepu
data = moveCoords(data, -2.5, -2.5, 0)	# mapa ve vrepu je -2.5 až 2.5
data = moveCoordsToObjectMiddle(data)

# pprint.pprint(data)

# for state in path:
#     pprint.pprint(state)
#     pprint.pprint(len(state))

# konec načtení cesty, napojuji se na vrep a postupně servíruji cíle pro UAV

uavCount = len(data['path'][0])
obstacles = data['map']['obstacles']

print('obstacles')
pprint.pprint(data['map']['obstacles'])
print('goals')
pprint.pprint(data['map']['goals'])
print('uav start')
pprint.pprint(data['path'][0])	# uav start
# print(len(obstacles))
# pprint.pprint(uavCount)
# for i in range(uavCount):
#     pprint.pprint('Quadricopter_target#' + str(i - 1))

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID == -1:
    sys.exit(1)


# vytvoření překážek
_, baseCuboid = vrep.simxGetObjectHandle(clientID, 'Cuboid', vrep.simx_opmode_oneshot_wait)

for obstacle in obstacles:
	vrep.shape
	_, [obstacleCuboid] = vrep.simxCopyPasteObjects(clientID, [baseCuboid], vrep.simx_opmode_oneshot_wait)
	vrep.object
	time.sleep(.5)	# při copypaste od sebe objekty odletí, počkám, než se "uklidní"
	# vrep.simxSetObjectPosition(clientID, obstacleCuboid, -1, [obstacle['location']['x'], obstacle['location']['y'], 0], vrep.simx_opmode_oneshot_wait)
	for i in range(1, 5):
		vrep.simxSetObjectPosition(clientID, obstacleCuboid, -1, [-0.995, -0.995, 0], vrep.simx_opmode_oneshot_wait)

# konec vytvoření překážek

x = 0.5
y = 0.5
z = 0.5

# _, temp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
# vrep.simxSetObjectPosition(clientID, temp, -1, [x, y, z], vrep.simx_opmode_oneshot_wait) # -1 je ku celé scéně, v tempu je handle, v podrtžítku návratová hodnota, oneshot wait čeká, než se příkaz provede

# vezmu handle targetů kvadrokoptér

targets = [0] * uavCount  # 2 je počet kvadrokoptér
uavs = [0] * uavCount  # 2 je počet kvadrokoptér

_, targets[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
_, uavs[0] = vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)

for i in range(uavCount):
    _, targets[i] = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#' + str(i - 1), vrep.simx_opmode_oneshot_wait)
    _, uavs[i] = vrep.simxGetObjectHandle(clientID, 'Quadricopter#' + str(i - 1), vrep.simx_opmode_oneshot_wait)


# vrep.simxSetObjectPosition(clientID, targets[0], -1, [x, y, z], vrep.simx_opmode_oneshot_wait)  # používat streaming nebo buffer místo oneshot wait, na první použít streaming a na další buffer
#
# vrep.simxGetObjectPosition(clientID, uavs[0], -1, vrep.simx_opmode_oneshot_wait)  # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl


