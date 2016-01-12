import vrep
import sys
import json
import math
import pprint
from utils import moveCoordsToObjectMiddle
from utils import moveCoords
from utils import resizeCoords
import time

print ('Program started')

# načtení cesty

with open('path5.json') as data_file:
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
targetDistanceFromUAV = 2


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

# vezmu handle targetů kvadrokoptér
for i, uavName in enumerate(uavNames):
	_, targets[uavIds[i]] = vrep.simxGetObjectHandle(clientID, targetNames[i], vrep.simx_opmode_oneshot_wait)
	_, uavs[uavIds[i]] = vrep.simxGetObjectHandle(clientID, uavNames[i], vrep.simx_opmode_oneshot_wait)


for stateId, state in enumerate(path):

	for id, uav in state.items():
		uavPosition = uav['pointParticle']['location']
		xEnd = uavPosition['x']
		yEnd = uavPosition['y']

		_, position = vrep.simxGetObjectPosition(clientID, uavs[id], -1, vrep.simx_opmode_oneshot_wait)  # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl

		xStart = position[0]
		yStart = position[1]

		x = xEnd - xStart
		y = yEnd - yStart

		distance = math.sqrt(x ** 2 + y ** 2)

		# pokud je vzdálenost cíle větší, než jsem si určil, normalizuji
		if distance > targetDistanceFromUAV:
			x = (x * targetDistanceFromUAV) / distance
			y = (y * targetDistanceFromUAV) / distance

		xTarget = xStart + x
		yTarget = yStart + y

		vrep.simxSetObjectPosition(clientID, targets[id], -1, [xTarget, yTarget, z], vrep.simx_opmode_oneshot_wait)  # používat streaming nebo buffer místo oneshot wait, na první použít streaming a na další buffer

	# print('current state:')
	# print(stateId)
	uavsReachedTargets = {}
	for id in uavIds:
		uavsReachedTargets[id] = False

	allUavsReachedTarget = False

	# print('allUavsReachedTarget')
	# print(allUavsReachedTarget)
	# print('uavsReachedTargets')
	# print(uavsReachedTargets)

	# čeká se, než se k dalšímu stavu dorazí, než se nastaví jako cíl

	# více navzorkovat cestu, nebo popsat analyticky. dávat uav cíl vždy s konstantní vzdáleností před UAV (perioda 0.2)
	# když pvní dorazí, dám další stav, udělám přímky mezi stavy a mezi stavy konst. čas letu. (pro vzdálenější cíl budu servírovat částečné cíle dále od sebe)
	# kromě 1. stavu brát vzdálenost mezi UAV a dalším stavem, místo 2 stavů
	# vzorkovat trajektorie ekvidistantně v čase
	while not allUavsReachedTarget:
		# input("Press Enter to continue...")
		for id, uav in state.items():
			uavPosition = uav['pointParticle']['location']
			# pprint.pprint('target position')
			# pprint.pprint(uavPosition)

			_, position = vrep.simxGetObjectPosition(clientID, uavs[id], -1, vrep.simx_opmode_oneshot_wait)  # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl
			# pprint.pprint('current position')
			# pprint.pprint(position)

			currentX = position[0]
			currentY = position[1]

			x = uavPosition['x']
			y = uavPosition['y']

			# print('difference')
			# print((currentX - x) ** 2 + (currentY - y) ** 2)
			uavsReachedTargets[id] = (currentX - x) ** 2 + (currentY - y) ** 2 < 17

		allUavsReachedTarget = True
		for uavId, reachedTarget in uavsReachedTargets.items():
			allUavsReachedTarget = allUavsReachedTarget and reachedTarget

		# time.sleep(0.005)


	print('all uavs now have new state as target')
