import vrep
import sys

print ('Program started')


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

targets = [0]*2 # 2 je počet kvadrokoptér
uavs = [0]*2 # 2 je počet kvadrokoptér

_, targets[0] =vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_oneshot_wait)
_, targets[1] =vrep.simxGetObjectHandle(clientID, 'Quadricopter_target#0', vrep.simx_opmode_oneshot_wait)
_, uavs[0] =vrep.simxGetObjectHandle(clientID, 'Quadricopter', vrep.simx_opmode_oneshot_wait)
_, uavs[1] =vrep.simxGetObjectHandle(clientID, 'Quadricopter#0', vrep.simx_opmode_oneshot_wait)


vrep.simxSetObjectPosition(clientID, targets[0], -1, [x, y, z], vrep.simx_opmode_oneshot_wait) # používat streaming nebo buffer místo oneshot wait, na první použít streaming a na další buffer

vrep.simxGetObjectPosition(clientID, uavs[0]) # tímhle získám momentální polohu kvadrokoptéry, podle toho nasazuji další cíl


