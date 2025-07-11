import numpy as np
import math
from queue import PriorityQueue

# The graph is represented by adjacency matrix
graph = [[1,1,1,0,1,1],[1,0,0,1,1,1],[1,1,0,1,1,1],[0,1,1,0,1,1],[1,1,1,1,0,1],[1,1,1,1,1,0]]
graph = np.array(graph)
print("Input Graph")
print(graph)

# A star Algorithm to find the optimal path
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
                priorityQueue.put(((estTotalCost[neighbour[0]][neighbour[1]]), neighbour))

    return None

def Neighbours(curNode, grid):
    row = grid.shape[0]
    column = grid.shape[1]
    list = []

    moveList = [[-1, -1], [-1, 0], [-1, 1], [0, -1], [0, 1], [1, -1], [1, 0], [1, 1]]
    for move in moveList:
        x = curNode[0]
        y = curNode[1]

        if (x + move[0]) >= 0 and (y + move[1]) >= 0 and (x + move[0]) < row and (y + move[1]) < column:
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
        return item == ele

def heuristic(node1, node2):
    temp1 = (node1[0] - node2[0]) ** 2
    temp2 = (node1[1] - node2[1]) ** 2
    return math.sqrt(temp1 + temp2)

# Run the A* algorithm
path1 = AStarSearchAlgo(graph, [3, 5], [0, 0])
print("Path")
print(path1)
