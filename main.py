import datetime
import vrep
import sys
import json
import math
import pprint
from utils import moveCoordsToObjectMiddle
from utils import moveCoords
from utils import resizeCoords
import time

now = datetime.datetime.now()
sys.stdout = open('log-' + str(now.hour) + '-' + str(now.minute) + '-' + str(now.second) + '.txt', 'w')		# redirecting output to file
print('Program started')

# načtení cesty

with open('path0.json') as data_file:
	data = json.load(data_file)

# pprint.pprint(data)

size = data['map']['size']
data = resizeCoords(data, size, 25)	# 1000 je velikost mapy při simulaci, 5 je velikost mapy ve vrepu
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
targetDistanceFromUAV = 0.2		# maximální vzdálenost z celého roje. Budu ostatním UAV nastavovat vzdálenost menší, aby je mohlo opožděné UAV dohnat, poměrově podle vzdáleností k cíli
distanceToNewState = 1.5

uavNames = ['Quadricopter']
targetNames = ['Quadricopter_target']
uavIds = []

if uavCount > 0:
	for i in range(1, uavCount):
		uavNames.append('Quadricopter#' + str(i - 1))
		targetNames.append('Quadricopter_target#' + str(i - 1))

for id in path[0]:
	uavIds.append(id)

# print(path[0])
# print(uavIds)

uavIds.sort()
targets = {}  # dictionary
uavs = {} 	# dictionary
newStates = {}
uavDummies = {}

uavDistanceFromTarget = {}
uavPositions = {}

uavInitialPositions = path[0]

# vezmu handle targetů kvadrokoptér
for i, uavName in enumerate(uavNames):
	_, targets[uavIds[i]] = vrep.simxGetObjectHandle(clientID, targetNames[i], vrep.simx_opmode_oneshot_wait)
	_, uavs[uavIds[i]] = vrep.simxGetObjectHandle(clientID, uavNames[i], vrep.simx_opmode_oneshot_wait)
	vrep.simxGetObjectPosition(clientID, uavs[uavIds[i]], -1, vrep.simx_opmode_streaming)

	# znázornění cíle
	_, newStates[uavIds[i]] = vrep.simxCreateDummy(clientID, 0.25, [0, 255-i*50, i*50], vrep.simx_opmode_oneshot_wait)
	# _, uavDummies[uavIds[i]] = vrep.simxCreateDummy(clientID, 0.15, [0, 255-i*50, i*50], vrep.simx_opmode_oneshot_wait)


for stateId, state in enumerate(path):

	for id, uav in state.items():
		uavPosition = uav['pointParticle']['location']
		xEnd = uavPosition['x']
		yEnd = uavPosition['y']
		vrep.simxSetObjectPosition(clientID, newStates[id], -1, [xEnd, yEnd, z], vrep.simx_opmode_oneshot)

	# print('current state:')
	# print(stateId)
	uavsReachedTargets = {}
	for id in uavIds:
		uavsReachedTargets[id] = False

	allUavsReachedTarget = False

	# čeká se, než se k dalšímu stavu dorazí, než se nastaví jako cíl

	# více navzorkovat cestu, nebo popsat analyticky. dávat uav cíl vždy s konstantní vzdáleností před UAV (perioda 0.2)
	# když pvní dorazí, dám další stav, udělám přímky mezi stavy a mezi stavy konst. čas letu. (pro vzdálenější cíl budu servírovat částečné cíle dále od sebe)
	# kromě 1. stavu brát vzdálenost mezi UAV a dalším stavem, místo 2 stavů
	# vzorkovat trajektorie ekvidistantně v čase

	while not allUavsReachedTarget:

		time.sleep(0.05)

		for id, uav in state.items():
			uavPosition = uav['pointParticle']['location']
			xEnd = uavPosition['x']
			yEnd = uavPosition['y']
			_, position = vrep.simxGetObjectPosition(clientID, uavs[id], -1, vrep.simx_opmode_buffer)  # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl
			xStart = position[0]
			yStart = position[1]

			uavPositions[id] = position
			# vrep.simxSetObjectPosition(clientID, uavDummies[id], -1, [xStart, yStart, z], vrep.simx_opmode_oneshot)

			#na začátku někdy VREP špatně načítá počíteční polohu UAV, proto místo něj nastavím počáteční pozici z jsonu
			if xStart == 0 and yStart == 0:
				xStart = uavInitialPositions[id]['pointParticle']['location']['x']
				yStart = uavInitialPositions[id]['pointParticle']['location']['y']

			x = xEnd - xStart
			y = yEnd - yStart

			distance = math.sqrt(x ** 2 + y ** 2)

			uavDistanceFromTarget[id] = distance

		maxDistance = max(uavDistanceFromTarget.values())
		# print('uavPositions: ')
		# pprint.pprint(uavPositions)
		# print('maxDistance')
		# print(maxDistance)

		for id, uav in state.items():
			position = uavPositions[id]
			xStart = position[0]
			yStart = position[1]

			uavPosition = uav['pointParticle']['location']
			xEnd = uavPosition['x']
			yEnd = uavPosition['y']

			x = xEnd - xStart
			y = yEnd - yStart

			distance = uavDistanceFromTarget[id]
			ratio = distance / maxDistance

			# pokud je vzdálenost cíle větší, než jsem si určil, normalizuji
			if distance > targetDistanceFromUAV:
				newDistance = ratio * targetDistanceFromUAV
				x = (x * newDistance) / distance
				y = (y * newDistance) / distance

			reducedDistance = math.sqrt(x ** 2 + y ** 2)

			xTarget = xStart + x
			yTarget = yStart + y

			vrep.simxSetObjectPosition(clientID, targets[id], -1, [xTarget, yTarget, z], vrep.simx_opmode_oneshot)
			print('position updated, start position of uav ' + id + '(vrep name: ' + uavNames[uavIds.index(id)] + '):')
			print('start: ' + str(xStart) + ', ' + str(yStart))
			print('end: ' + str(xEnd) + ', ' + str(yEnd))
			print('movement: ' + str(x) + ', ' + str(y))
			print('distance: ' + str(distance))
			print('reduced distance: ' + str(reducedDistance))

			uavsReachedTargets[id] = distance < distanceToNewState

		allUavsReachedTarget = True
		for uavId, reachedTarget in uavsReachedTargets.items():
			allUavsReachedTarget = allUavsReachedTarget and reachedTarget


	print('all uavs now have new state as target')

print('uavs arrived to target')