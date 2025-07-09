from PIL import Image
import numpy as np
import math
import random
from bresenham import bresenham
import networkx as nx
import matplotlib.pyplot as plt

# Load occupancy grid map
occupancy_map_img = Image.open("occupancy_map.png")
occupancy_grid = (np.asarray(occupancy_map_img) > 0).astype(int)

# Dictionary to store node occupancy status
vertex = {}

# Initialize PRM graph
prmGraph = nx.Graph()

def init_vertex_map(grid):
    """Convert occupancy grid into a vertex dictionary"""
    global vertex
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            vertex[(row, col)] = grid[row][col]

def is_occupied(v):
    """Check if node is occupied"""
    return vertex[v] == 0  # 0 means occupied (black pixel)

def euclidean_distance(p1, p2):
    """Compute Euclidean distance between two points"""
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def is_path_free(v1, v2):
    """Check if all intermediate points between two nodes are free"""
    for v in bresenham(v1[0], v1[1], v2[0], v2[1]):
        if is_occupied(v):
            return False
    return True

def is_within_distance(p1, p2, threshold):
    """Check if two nodes are within connection distance"""
    return euclidean_distance(p1, p2) <= threshold

def get_random_free_vertex():
    """Generate a random free vertex from the map"""
    while True:
        r = random.randint(0, occupancy_grid.shape[0] - 1)
        c = random.randint(0, occupancy_grid.shape[1] - 1)
        if not is_occupied((r, c)):
            return (r, c)

def add_vertex_to_prm(graph, new_vertex, max_distance):
    """Add a new vertex to the PRM and connect to nearby vertices"""
    node_id = graph.number_of_nodes() + 1
    graph.add_node(node_id, pos=new_vertex)
    
    for node in graph.nodes:
        existing_pos = graph.nodes[node]['pos']
        if is_within_distance(existing_pos, new_vertex, max_distance) and is_path_free(existing_pos, new_vertex):
            dist = euclidean_distance(existing_pos, new_vertex)
            graph.add_edge(node, node_id, weight=dist)
    
    return node_id, graph

def construct_prm(graph, num_nodes, max_distance):
    """Build the Probabilistic Roadmap graph"""
    for _ in range(num_nodes):
        new_vertex = get_random_free_vertex()
        _, graph = add_vertex_to_prm(graph, new_vertex, max_distance)
    return graph

def plot_prm_with_path(graph, path, size):
    """Visualize PRM graph and path on the occupancy grid"""
    plt.figure(figsize=size)
    plt.imshow(occupancy_grid, cmap='gray')

    for (n1, n2) in graph.edges:
        p1, p2 = graph.nodes[n1]['pos'], graph.nodes[n2]['pos']
        plt.plot([p1[1], p2[1]], [p1[0], p2[0]], 'b-', linewidth=1)

    path_array = np.array(path)
    plt.plot(path_array[:, 1], path_array[:, 0], 'r-', linewidth=2)

    plt.title("PRM Path Planning")
    plt.show()

# ---- Main Execution ----

init_vertex_map(occupancy_grid)

# Parameters
N = 2500            # Number of sampled vertices
d_max = 75          # Maximum connection distance

# Construct the roadmap
prmGraph = construct_prm(prmGraph, N, d_max)

# Add start and goal manually
start_node, prmGraph = add_vertex_to_prm(prmGraph, (635, 140), d_max)
goal_node, prmGraph = add_vertex_to_prm(prmGraph, (350, 400), d_max)

# Run A* pathfinding
path_nodes = nx.astar_path(prmGraph, source=start_node, target=goal_node)
path_coords = [prmGraph.nodes[n]['pos'] for n in path_nodes]

# Output and visualize
print("Path:", path_coords)
plot_prm_with_path(prmGraph, path_coords, (5, 5))
