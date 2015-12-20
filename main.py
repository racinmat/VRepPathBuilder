import vrep
import sys
import json
import pprint
from utils import moveCoordsToObjectMiddle
from utils import moveCoords
from utils import resizeCoords
import time

print ('Program started')

# načtení cesty

with open('path.json') as data_file:
    data = json.load(data_file)

# pprint.pprint(data)

data = resizeCoords(data, 1000, 25)	# 1000 je velikost mapy při simulaci, 5 je velikost mapy ve vrepu
data = moveCoords(data, -12.5, -12.5, 0)	# mapa ve vrepu je -2.5 až 2.5
data = moveCoordsToObjectMiddle(data)
path = data['path']

# pprint.pprint(data)

# for state in path:
#     pprint.pprint(state)
#     pprint.pprint(len(state))

# konec načtení cesty, napojuji se na vrep a postupně servíruji cíle pro UAV

uavCount = len(path[0])

# print(len(obstacles))
# pprint.pprint(uavCount)
# for i in range(uavCount):
#     pprint.pprint('Quadricopter_target#' + str(i - 1))

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)  # Connect to V-REP
if clientID == -1:
    sys.exit(1)


z = 0.511

# _, temp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
# vrep.simxSetObjectPosition(clientID, temp, -1, [x, y, z], vrep.simx_opmode_oneshot_wait) # -1 je ku celé scéně, v tempu je handle, v podrtžítku návratová hodnota, oneshot wait čeká, než se příkaz provede

# vezmu handle targetů kvadrokoptér

uavNames = ['Quadricopter']
targetNames = ['Quadricopter_target']
uavIds = []

if uavCount > 0:
	for i in range(1, uavCount):
		uavNames.append('Quadricopter#' + str(i - 1))
		targetNames.append('Quadricopter_target#' + str(i - 1))

for id in path[0]:
	uavIds.append(id)

print(path[0])
print(uavIds)

uavIds.sort()
targets = {}  # dictionary
uavs = {} 	# dictionary

for i, uavName in enumerate(uavNames):
	_, targets[uavIds[i]] = vrep.simxGetObjectHandle(clientID, targetNames[i], vrep.simx_opmode_oneshot_wait)
	_, uavs[uavIds[i]] = vrep.simxGetObjectHandle(clientID, uavNames[i], vrep.simx_opmode_oneshot_wait)


for stateId, state in enumerate(path):

	for id, uav in state.items():
		uavPosition = uav['pointParticle']['location']
		x = uavPosition['x']
		y = uavPosition['y']
		vrep.simxSetObjectPosition(clientID, targets[id], -1, [x, y, z], vrep.simx_opmode_oneshot_wait)  # používat streaming nebo buffer místo oneshot wait, na první použít streaming a na další buffer

	print('current state:')
	print(stateId)
	uavsReachedTargets = {}
	for id in uavIds:
		uavsReachedTargets[id] = False

	allUavsReachedTarget = False

	print('allUavsReachedTarget')
	print(allUavsReachedTarget)
	print('uavsReachedTargets')
	print(uavsReachedTargets)

	# čeká se, než se k dalšímu stavu dorazí, než se nastaví jako cíl

	while not allUavsReachedTarget:
		# input("Press Enter to continue...")
		for id, uav in state.items():
			uavPosition = uav['pointParticle']['location']
			pprint.pprint('target position')
			pprint.pprint(uavPosition)

			_, position = vrep.simxGetObjectPosition(clientID, uavs[id], -1, vrep.simx_opmode_oneshot_wait)  # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl
			pprint.pprint('current position')
			pprint.pprint(position)

			currentX = position[0]
			currentY = position[1]

			x = uavPosition['x']
			y = uavPosition['y']

			print('difference')
			print((currentX - x) ** 2 + (currentY - y) ** 2)
			uavsReachedTargets[id] = (currentX - x) ** 2 + (currentY - y) ** 2 < 0.1

		allUavsReachedTarget = True
		for uavId, reachedTarget in uavsReachedTargets.items():
			allUavsReachedTarget = allUavsReachedTarget and reachedTarget

		time.sleep(.2)


	print('all uavs now have new state as target')
