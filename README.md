# Path Planning Algorithms in Occupancy Grids: PRM & A* in Python

This repository provides three Python-based implementations for path planning on a 2D binary occupancy map. These techniques are fundamental in robotics and AI for autonomous navigation in known environments. The implementations include:

1. **Probabilistic Roadmap (PRM) with NetworkX**
2. **A\* Search on an Occupancy Grid**
3. **A\* Search on an Adjacency Matrix**

These algorithms are fundamental to robotics and AI for tasks like autonomous navigation, obstacle avoidance, and roadmap-based planning.

---

## Code Overview

### 1. `prm_graph_planner.py`
> **Probabilistic Roadmap (PRM) with Graph Construction**

This script constructs a sparse roadmap using randomly sampled free-space points and connects them using line-of-sight checks via Bresenham’s algorithm. The roadmap is represented using a `networkx.Graph`, and path planning is performed using A\* on this graph.

- **Key Features:**
  - Random sampling of collision-free nodes
  - Connection of nodes within a given radius (`dmax`)
  - Euclidean distance used as a metric
  - Graph visualization with roadmap and path overlay
- **Libraries Used:** `networkx`, `PIL`, `matplotlib`, `numpy`, `bresenham`
- **Input:** Binary occupancy grid image (`occupancy_map.png`)
- **Use Case:** Efficient path planning in large, sparsely structured environments

---

### 2. `astar_grid_planner.py`
> **A\* Algorithm on Binary Occupancy Grid**

This implementation performs A\* search directly on the occupancy matrix (2D grid) with 8-connected neighbors. It uses a cost matrix and priority queue to track the optimal path.

- **Key Features:**
  - Works directly on pixel-level grid
  - Uses Euclidean distance as the heuristic
  - Handles arbitrary start and goal positions
  - Path visualization on top of the map
- **Libraries Used:** `numpy`, `math`, `matplotlib`, `queue`, `PIL`
- **Input:** Binary image as occupancy grid (`occupancy_map.png`)
- **Use Case:** Precise pixel-level planning with fine resolution

---

### 3. `astar_adjacency_matrix.py`
> **A\* Search Using a Custom Adjacency Matrix**

This script implements A\* on a user-defined adjacency matrix representing connectivity between abstract states. Unlike a grid, it operates on a logical graph structure defined by a 2D array.

- **Key Features:**
  - Works on any graph represented as an adjacency matrix
  - Custom start and goal nodes
  - 8-connected neighbor search logic
  - Euclidean and Manhattan heuristics
- **Libraries Used:** `numpy`, `math`, `queue`
- **Input:** Hardcoded adjacency matrix
- **Use Case:** Abstract graph-based pathfinding (e.g., custom maps or logic graphs)

---

## Input Format

- **Occupancy Grid** (`occupancy_map.png`): Black pixels represent obstacles (0), white pixels represent free space (1).
- Image is converted using:  
  ```python
  occupancy_grid = (np.asarray(Image.open('occupancy_map.png')) > 0).astype(int)

## Requirements

Make sure to install the following Python packages:

```bash
pip install numpy matplotlib networkx pillow bresenham

## To Run Scripts

Use the following commands to run the individual scripts:

```bash
python prm_graph_planner.py
python astar_grid_planner.py
python astar_adjacency_matrix.py
