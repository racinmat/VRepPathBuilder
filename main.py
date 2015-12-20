import vrep
import sys
import json
import pprint

print ('Program started')

# načtení cesty

with open('path.json') as data_file:
    path = json.load(data_file)

# pprint.pprint(path)

for state in path:
    pprint.pprint(state)
    pprint.pprint(len(state))

# konec načtení cesty, napojuji se na vrep a postupně servíruji cíle pro UAV

uavCount = len(path[0])

# jen nějaké testy
pprint.pprint(uavCount)
for i in range(uavCount):
    pprint.pprint('Quadricopter_target#' + str(i - 1))
# konec testů

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to V-REP
if clientID == -1:
    sys.exit(1)

x = 0.5
y = 0.5
z = 0.5

_, temp = vrep.simxCreateDummy(clientID, 0.1, None, vrep.simx_opmode_oneshot_wait)
vrep.simxSetObjectPosition(clientID, temp, -1, [x, y, z], vrep.simx_opmode_oneshot_wait) # -1 je ku celé scéně, v tempu je handle, v podrtžítku návratová hodnota, oneshot wait čeká, než se příkaz provede

# vezmu handle targetů kvadrokoptér

targets = [0] * uavCount # 2 je počet kvadrokoptér
uavs = [0] * uavCount # 2 je počet kvadrokoptér

_, targets[0] =vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
_, uavs[0] =vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)

for i in range(uavCount):
    _, targets[i] =vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#' + str(i - 1), vrep.simx_opmode_oneshot_wait)
    _, uavs[i] =vrep.simxGetObjectHandle(clientID, 'Quadricopter#' + str(i - 1), vrep.simx_opmode_oneshot_wait)


vrep.simxSetObjectPosition(clientID, targets[0], -1, [x, y, z], vrep.simx_opmode_oneshot_wait) # používat streaming nebo buffer místo oneshot wait, na první použít streaming a na další buffer

vrep.simxGetObjectPosition(clientID, uavs[0]) # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl


