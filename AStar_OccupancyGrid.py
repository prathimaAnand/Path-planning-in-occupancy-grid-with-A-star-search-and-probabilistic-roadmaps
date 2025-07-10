import numpy as np
import math
import matplotlib.pyplot as plt
from queue import PriorityQueue
import PIL

# Load and process the occupancy map image
occupancy_map_img = PIL.Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

print("Occupancy Grid")
print(occupancy_grid)

def AStarSearchAlgo(grid, start, end):
    columns = grid.shape[1]
    rows = grid.shape[0]

    pred = np.empty((rows, columns), dtype=object)
    costTo = np.full((rows, columns), math.inf)
    estTotalCost = np.full((rows, columns), math.inf)

    costTo[start[0]][start[1]] = 0
    estTotalCost[start[0]][start[1]] = heuristic(start, end)
    priorityQueue = PriorityQueue()
    priorityQueue.put((heuristic(start, end), (start)))

    while not priorityQueue.empty():
        curNode = priorityQueue.get()[1]

        if curNode == end:
            return RecoverPath(start, end, pred)[::-1]

        for neighbour in Neighbours(curNode, grid):
            finalCost = costTo[curNode[0]][curNode[1]] + heuristic(curNode, neighbour)

            if finalCost < costTo[neighbour[0]][neighbour[1]]:
                pred[neighbour[0]][neighbour[1]] = curNode
                costTo[neighbour[0]][neighbour[1]] = finalCost
                estTotalCost[neighbour[0]][neighbour[1]] = finalCost + heuristic(neighbour, end)

                if queueCheck(priorityQueue.queue, neighbour):
                    priorityQueue.queue.remove(neighbour)
                priorityQueue.put((estTotalCost[neighbour[0]][neighbour[1]], neighbour))
    return None

def Neighbours(curNode, grid):
    row = grid.shape[0]
    column = grid.shape[1]
    list = []
    moveList = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]

    for move in moveList:
        x, y = curNode[0], curNode[1]
        if (0 <= x + move[0] < row) and (0 <= y + move[1] < column):
            if grid[x + move[0]][y + move[1]] != 0:
                list.append([x + move[0], y + move[1]])
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
    for ele in queue:
        if item == ele[1]:
            return True
    return False

def heuristic(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

def plot(grid, path, size=(5, 5)):
    rows, columns = grid.shape
    plt.figure(figsize=size)
    for i in range(rows):
        for j in range(columns):
            if grid[i][j] == 0:
                plt.plot(i, j, 'ko')
            elif [i, j] in path:
                plt.plot(i, j, 'ro')
            else:
                plt.plot(i, j, 'w.')
    plt.show()

# Example usage
start = [835, 140]
goal = [350, 400]
path1 = AStarSearchAlgo(occupancy_grid, start, goal)
print("Path")
print(path1)
print("Length:", len(path1))
plot(occupancy_grid, path1, size=(5, 5))
