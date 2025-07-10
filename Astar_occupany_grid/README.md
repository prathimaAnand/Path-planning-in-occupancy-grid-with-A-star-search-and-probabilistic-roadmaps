# A* Search on Occupancy Grid Map from Image (2b)

This script performs route planning using the A* search algorithm on a binary occupancy grid generated from an image.

## Problem Statement

Given:
- An image representing an occupancy map (0 = occupied, 1 = free)
- Start and goal coordinates
Apply A* algorithm using:
- 8-connected neighbors
- Euclidean distance as both edge cost and heuristic

## Features

- **Input**: Binary occupancy grid from an image
- **Search**: 8-connected A* search
- **Visualization**: Matplotlib plotting of grid and path
- **Heuristic**: Euclidean distance
- **Cost Function**: Euclidean edge distance

---

## Output Example

```
Occupancy Grid
[[...]]
Path
[[835, 140], [834, 141], ..., [350, 400]]
Length: 725
```

## Skills Demonstrated

- Python implementation of search algorithm
- Image processing with `PIL`
- NumPy and Matplotlib for grid handling and visualization
- A* path planning and Euclidean heuristics

## Dependencies

```bash
pip install numpy pillow matplotlib
```

---

## How to Run

Place your map as `occupancy_map.png` in the working directory and run:

```bash
python AStar_OccupancyGrid.py
```

---

## References

- [A* Search - Wikipedia](https://en.wikipedia.org/wiki/A*_search_algorithm)
- [Python Imaging Library (PIL)](https://pillow.readthedocs.io/en/stable/)
- [Matplotlib Docs](https://matplotlib.org/)
