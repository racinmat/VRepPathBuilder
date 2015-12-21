import json

var = 2
print(var)
arrVar = [2]
print(arrVar)
print(len(arrVar))
print(*arrVar)

arr = {}
arr[44] = 'a'
arr[45] = 'b'
arr[46] = 'c'
arr[47] = 'd'
for key, element in arr.items():
	print(element)


uavCount = 4
uavNames = ['Quadricopter']
targetNames = ['Quadricopter_target']
if uavCount > 0:
	for i in range(1, uavCount):
		uavNames.append('Quadricopter#' + str(i - 1))
		targetNames.append('Quadricopter_target#' + str(i - 1))

for id, e in enumerate(uavNames):
	print(id)
	print(e)

with open('path3.json') as data_file:
    data = json.load(data_file)

# pprint.pprint(data)

path = data['path']
print(path)

print(type(path).__name__)

for stateId, state in enumerate(path):
	print(stateId)
