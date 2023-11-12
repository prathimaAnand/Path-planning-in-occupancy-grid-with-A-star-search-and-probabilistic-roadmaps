from PIL import Image, ImageDraw
import numpy as np
import math
import random
from bresenham import bresenham
import networkx as nx
import matplotlib.pyplot as plt
occupancy_map_img = Image.open("occupancy_map.png")
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)
print('Occupancy Grid')
print(occupancy_grid)
vertex = {}
prmGraph = nx.Graph()
def nodeOccupiedCheck(v,vertex):
return(vertex[v]==0)
def euclidianDistance(x1, x2, v1,v2):
temp1 = ((v2 - v1) ** 2)
temp2 = ((x2 - x1) **2)
euclidianDistance = math.sqrt(temp1 + temp2)
return euclidianDistance
def pathCheck(v1,v2,vertex):
pathList = list(bresenham(v1[0],v1[1],v2[0],v2[1]))
for v in pathList:
if nodeOccupiedCheck(v,vertex):
return False
return True
def plot(grid, path, size):
rows = grid.shape[0]
columns = grid.shape[1]
plt.figure(figsize=size)
for i in range(rows):
for j in range(columns):
if grid[i][j] == 0:
mpr2c:
plt.plot(i, j, 'ko')
elif [i,j] in path:
plt.plot(i, j, 'ro')
else:
plt.plot(i, j, 'w.')
plt.ion()
plt.show()
plt.savefig( str(size[0])+"x"+str(size[1])+'.png')
def plot2(graph, path, size):
plt.figure(figsize=(20,20))
plt.imshow(occupancy_grid, cmap ='gray')
for (e1,e2) in prmGraph.edges:
plt.plot((graph.nodes[e1]['pos'][1],graph.nodes[e2]['pos'][1]),
(graph.nodes[e1]['pos'][0],graph.nodes[e2]['pos'][0]),
linewidth=1,
color='blue')
data = np.array(path)
plt.plot(data[:,1],data[:,0], color='red')
plt.show()
def distanceCheck(v1,v2,connection_distance):
x1,v1 = v1
x2,v2 = v2
return(euclidianDistance(x1,x2,v1,v2)<=connection_distance)
def getNewVertex(vertex):
global occupancy_grid
while True:
newVertex = (int(np.random.uniform(low=0, high=occupancy_grid.shape[0])),
int(np.random.uniform(low=0, high=occupancy_grid.shape[1])))
if not nodeOccupiedCheck(newVertex,vertex):
return newVertex
def addVertex(prmGraph,newVertex,dmax):
global vertex
prmGraph.add_node(prmGraph.number_of_nodes()+1, pos = newVertex)
newVertexPosition = prmGraph.number_of_nodes()
for node in prmGraph:
v = prmGraph.nodes[node]['pos']
if (distanceCheck(v,newVertex,dmax) and pathCheck(v,newVertex,vertex)):
x1,v1 = v
x2,v2 = newVertex
prmGraph.add_edge(node,newVertexPosition,weight = euclidianDistance(x1,x2,v1,v2))
return newVertexPosition,prmGraph
def constructPrm(prmGraph,N,dmax):
global vertex
for k in range(N):
newVertex = getNewVertex(vertex)
newVertexPosition,prmGraph = addVertex(prmGraph,newVertex,dmax)
return(prmGraph)
def initfunction(occupancy_grid):
global vertex
for row in range(occupancy_grid.shape[0]):
for col in range(occupancy_grid.shape[1]):
vertex[(row,col)] = occupancy_grid[row][col]
initfunction(occupancy_grid)
N = 2500
dmax =75
prmGraph = constructPrm(prmGraph,N,dmax)
startNode,prmGraph = addVertex(prmGraph,(635,140),dmax)
goalNode,prmGraph = addVertex(prmGraph,(350,400),dmax)
path_nodes = nx.astar_path(prmGraph,source = startNode, target = goalNode)
path=[]
for nodes in path_nodes:
path.append(prmGraph.nodes[nodes]['pos'])
print(path)
plot2(prmGraph, path, (5,5))
