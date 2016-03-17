import json
import pprint
from utils import moveCoordsToObjectMiddle
from utils import moveCoords
from utils import resizeCoords

print ('Program started')

# načtení cesty

with open('path8.json') as data_file:
    data = json.load(data_file)

# pprint.pprint(data)

size = data['map']['size']

data = resizeCoords(data, size, 25)	# 1000 je velikost mapy při simulaci, 5 je velikost mapy ve vrepu
data = moveCoords(data, -12.5, -12.5, 0)	# mapa ve vrepu je -2.5 až 2.5
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