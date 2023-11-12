import numpy as np
import math
import matplotlib.pyplot as plt
from queue import PriorityQueue
import PIL
occupancy_map_img = PIL.Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
print('Occupancy Grid')
print(occupancy_grid)
# A star Algorithm to find the optimal path
def AStarSearchAlgo(grid, start, end):
# Get the dimentions of the matrix
columns = grid.shape[1]
rows = grid.shape[0]
# Initial values are set to infinity
pred = np.empty((rows,columns), dtype = object)
costTo = np.full((rows,columns), math.inf)
estTotalCost = np.full((rows,columns), math.inf)
costTo[start[0]][start[1]] = 0
estTotalCost[start[0]][start[1]] = heuristic(start, end)
priorityQueue = PriorityQueue()
priorityQueue.put( (heuristic(start, end),(start)))
while not priorityQueue.empty():
curNode = priorityQueue.get()[1]
if curNode == end:
return RecoverPath(start, end, pred)[::-1]
for neighbour in Neighbours(curNode, grid):
finalCost = costTo[curNode[0]][curNode[1]] + heuristic(curNode , neighbour)
if finalCost < costTo[neighbour[0]][neighbour[1]]:
pred[neighbour[0]][neighbour[1]] = curNode
costTo[neighbour[0]][neighbour[1]] = finalCost
estTotalCost[neighbour[0]][neighbour[1]] = finalCost + heuristic(neighbour , end)
Code for 26.-Route planning in occupancy
grid with A scarce
if queueCheck(priorityQueue,neighbour):
priorityQueue.queue.remove(neighbour)
priorityQueue.put(((estTotalCost[neighbour[0]][neighbour[1]]),(neighbour)))
return None
def Neighbours(curNode, grid):
row = grid.shape[0]
column= grid.shape[1]
list = []
moveList =[[-1, -1],[-1, 0],[-1, 1],[0, -1],[0, 1],[1, -1],[1, 0],[1 ,1]]
for move in moveList:
x= curNode[0]
y= curNode[1]
if (x + move[0] >=0) and (y + move[1] >=0) and (x+move[0]<row) and (y + move[1]
<column):
if grid[x + move[0]][y + move[1]]!=0:
list.append([x + move[0] , y + move[1]])
return list
def RecoverPath(p1, p2, pred):
path = []
current = p2
while current != p1:
path.append(current)
current = pred[current[0]][current[1]]
path.append(p1)
return path
def queueCheck(queue, item):
for ele in queue.queue:
return item == ele
def plot(grid, path, size):
rows = grid.shape[0]
columns = grid.shape[1]
plt.figure(figsize=size)
for i in range(rows):
for j in range(columns):
if grid[i][j] == 0:
plt.plot(i, j, 'ko')
elif [i,j] in path:
plt.plot(i, j, 'ro')
else:
plt.plot(i, j, 'w.')
plt.show()
plt.savefig( str(size[0])+"x"+str(size[1])+'.png')
def heuristic(node1, node2):
temp1 = (node1[0]-node2[0])**2
temp2 = (node1[1]-node2[1])**2
return math.sqrt(temp1 + temp2)
def heuristic2(node1, node2):
return (abs(node1[0]-node2[0]) + abs(node1[1]-node2[1]))
path1 = AStarSearchAlgo(occupancy_grid, [635,140], [350, 400])
print("Path")
print(path1)
print("length "+str(len(path1)))
plot(occupancy_grid, path1, size = (5,5))
