import numpy as np
import math
import matplotlib.pyplot as plt
from queue import PriorityQueue
from PIL import Image

# Load and convert image to occupancy grid (1 = free, 0 = occupied)
occupancy_map_img = Image.open('occupancy_map.png')
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

print('Occupancy Grid')
print(occupancy_grid)

# A star Algorithm to find the optimal path
def AStarSearchAlgo(grid, start, end):
    # Get the dimensions of the matrix
    columns = grid.shape[1]
    rows = grid.shape[0]

    # Initial values are set to infinity
    pred = np.empty((rows, columns), dtype=object)
    costTo = np.full((rows, columns), math.inf)           # g(n): Cost from start
    estTotalCost = np.full((rows, columns), math.inf)     # f(n): Estimated total cost

    costTo[start[0]][start[1]] = 0
    estTotalCost[start[0]][start[1]] = heuristic(start, end)

    priorityQueue = PriorityQueue()
    priorityQueue.put((estTotalCost[start[0]][start[1]], start))

    while not priorityQueue.empty():
        curNode = priorityQueue.get()[1]

        # Check if goal reached
        if curNode == end:
            return RecoverPath(start, end, pred)[::-1]

        # Explore neighbors
        for neighbour in Neighbours(curNode, grid):
            finalCost = costTo[curNode[0]][curNode[1]] + heuristic(curNode, neighbour)

            # If better path found
            if finalCost < costTo[neighbour[0]][neighbour[1]]:
                pred[neighbour[0]][neighbour[1]] = curNode
                costTo[neighbour[0]][neighbour[1]] = finalCost
                estTotalCost[neighbour[0]][neighbour[1]] = finalCost + heuristic(neighbour, end)

                # Remove existing entry from queue if present and re-insert with updated cost
                if queueCheck(priorityQueue, neighbour):
                    priorityQueue.queue = [q for q in priorityQueue.queue if q[1] != neighbour]
                priorityQueue.put((estTotalCost[neighbour[0]][neighbour[1]], neighbour))

    # Return None if path not found
    return None

# Function to get all the possible neighbors
def Neighbours(curNode, grid):
    row = grid.shape[0]
    column = grid.shape[1]
    neighbors = []

    # all 8 connected directions
    moveList = [[-1, -1], [-1, 0], [-1, 1],
                [0, -1],           [0, 1],
                [1, -1], [1, 0], [1, 1]]

    for move in moveList:
        x = curNode[0] + move[0]
        y = curNode[1] + move[1]
        if 0 <= x < row and 0 <= y < column:
            if grid[x][y] != 0:  # not occupied
                neighbors.append([x, y])
    return neighbors

# Function to recover the path from the predecessor list
def RecoverPath(p1, p2, pred):
    path = []
    current = p2
    while current != p1:
        path.append(current)
        current = pred[current[0]][current[1]]
        if current is None:
            return []  # No path found
    path.append(p1)
    return path

# Function to check if the element is present in the priority queue
def queueCheck(queue, item):
    for _, val in queue.queue:
        if val == item:
            return True
    return False

# Function to plot the map and the path
def plot(grid, path, size):
    rows = grid.shape[0]
    columns = grid.shape[1]
    plt.figure(figsize=size)

    for i in range(rows):
        for j in range(columns):
            if grid[i][j] == 0:
                plt.plot(j, i, 'ks')  # obstacle
            elif [i, j] in path:
                plt.plot(j, i, 'ro')  # path
            else:
                plt.plot(j, i, 'w.')  # free space

    plt.title("A* Path on Occupancy Grid")
    plt.gca().invert_yaxis()
    plt.tight_layout()
    plt.show()
    plt.savefig(f"{size[0]}x{size[1]}.png")

# Euclidean Distance Heuristic Function
def heuristic(node1, node2):
    temp1 = (node1[0] - node2[0]) ** 2
    temp2 = (node1[1] - node2[1]) ** 2
    return math.sqrt(temp1 + temp2)

# Manhattan Distance Heuristic Function (not used here)
def heuristic2(node1, node2):
    return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

# Run A* algorithm
path1 = AStarSearchAlgo(occupancy_grid, [635, 140], [350, 400])

print("Path")
print(path1)
print("Length " + str(len(path1)))

# Plot the result
plot(occupancy_grid, path1, size=(5, 5))
