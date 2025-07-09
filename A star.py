import numpy as np
import math
from queue import PriorityQueue

# Input Graph: 1 = accessible, 0 = obstacle
# The graph is represented by adjacency matrix
graph = [
    [1, 1, 1, 1, 0, 1],
    [1, 0, 1, 0, 1, 1],
    [1, 1, 0, 1, 0, 1],
    [1, 1, 1, 1, 0, 1],
    [0, 1, 0, 1, 1, 1],
    [1, 1, 1, 0, 1, 1]
]
graph = np.array(graph)
print('Input Graph')
print(graph)

# A star Algorithm to find the optimal path
def AStarSearchAlgo(grid, start, end):
    # Get the dimensions of the matrix
    columns = grid.shape[1]
    rows = grid.shape[0]

    # Initial values are set to infinity
    pred = np.empty((rows, columns), dtype=object)  # To store predecessors for path reconstruction
    costTo = np.full((rows, columns), math.inf)     # g(n): Cost from start to node
    estTotalCost = np.full((rows, columns), math.inf)  # f(n): Estimated total cost (g + h)

    costTo[start[0]][start[1]] = 0
    estTotalCost[start[0]][start[1]] = heuristic(start, end)

    priorityQueue = PriorityQueue()
    priorityQueue.put((estTotalCost[start[0]][start[1]], start))  # Start node with f-score

    while not priorityQueue.empty():
        curNode = priorityQueue.get()[1]

        # Goal check
        if curNode == end:
            return RecoverPath(start, end, pred)[::-1]  # Return reversed path

        # Explore neighbors
        for neighbour in Neighbours(curNode, grid):
            finalCost = costTo[curNode[0]][curNode[1]] + heuristic(curNode, neighbour)

            # Update costs and predecessor if a better path is found
            if finalCost < costTo[neighbour[0]][neighbour[1]]:
                pred[neighbour[0]][neighbour[1]] = curNode
                costTo[neighbour[0]][neighbour[1]] = finalCost
                estTotalCost[neighbour[0]][neighbour[1]] = finalCost + heuristic(neighbour, end)

                # Remove and reinsert to update priority (no native decrease-key)
                if inQueue(priorityQueue, neighbour):
                    priorityQueue.queue = [item for item in priorityQueue.queue if item[1] != neighbour]
                priorityQueue.put((estTotalCost[neighbour[0]][neighbour[1]], neighbour))

    # No path found
    return None

# Function to get all the possible neighbors
def Neighbours(curNode, grid):
    row = grid.shape[0]
    column = grid.shape[1]
    list = []
    # all 8 connected directions
    moveList = [[-1, -1], [-1, 0], [-1, 1],
                [0, -1],           [0, 1],
                [1, -1], [1, 0],   [1, 1]]

    for move in moveList:
        x = curNode[0] + move[0]
        y = curNode[1] + move[1]

        # Check within bounds and that it's not an obstacle
        if 0 <= x < row and 0 <= y < column:
            if grid[x][y] != 0:
                list.append([x, y])
    return list

# Function to recover the path from the predecessor list
def RecoverPath(p1, p2, pred):
    path = []
    current = p2
    while current != p1:
        path.append(current)
        current = pred[current[0]][current[1]]
        if current is None:  # Path not reachable
            return []
    path.append(p1)
    return path

# Function to check if the element is present in the priority queue
def inQueue(queue, item):
    for _, val in queue.queue:
        if val == item:
            return True
    return False

# Euclidean Distance Heuristic Function
def heuristic(node1, node2):
    temp1 = (node1[0] - node2[0]) ** 2
    temp2 = (node1[1] - node2[1]) ** 2
    return math.sqrt(temp1 + temp2)

# Manhattan Distance Heuristic Function (Not used but available)
def heuristic2(node1, node2):
    return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

# Example Test
path1 = AStarSearchAlgo(graph, [3, 5], [0, 0])
print("Path")
print(path1)
