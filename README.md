# Bi-RRT-Connect-3D-Motion-Planner
A Bi-Directional RRT-Connect motion planner for continuous 3D environments, featuring sub-0.033s dynamic fast-replanning and vectorized AABB collision checking.

<img width="320" height="240" alt="test_maze" src="https://github.com/user-attachments/assets/b21fceec-7a50-4dc5-baab-8c9b1deb4c31" />


## Overview
This repository contains a highly optimized motion planning architecture designed to navigate continuous 3-D Euclidean spaces populated with static and dynamic axis-aligned bounding box (AABB) obstacles. 

The planner is engineered to solve both complex static environments (avoiding local minima / bug traps) and extreme dynamic scenarios requiring ultra-fast replanning for moving targets.

### 🏆 Key Performance Highlights
* **Dynamic Fast-Replanning:** Executes entirely from scratch in milliseconds, successfully satisfying strict sub-0.1s replanning limits for moving targets.
* **Robust Exploration:** The Bi-Directional RRT-Connect algorithm seamlessly escapes complex local minima (e.g., Maze environments).
* **Vectorized Collision Checking:** Replaced standard iterative collision loops with a purely vectorized NumPy implementation of the Slab Method, evaluating thousands of continuous line-segment-to-AABB connections simultaneously.

## Dependencies
- Python 3.11
- NumPy
- Matplotlib

## Project Structure
```text
Project/
    ├── maps/         # directory for storing map definitions
    ├── README.md           
    ├── main.py       # Execution script and visualization logic
    └── Planner.py    # Core algorithmic implementations
```

## Algorithmic Implementations
Inside `Planner.py`, two distinct approaches are provided:

1. MyPlanner (Default / Production):
The finalized Bi-Directional RRT-Connect algorithm. It successfully solves all environments, effortlessly escaping bug traps in highly constrained spaces while maintaining millisecond execution times.

2. GoalBiasedPlanner (Reference):
A unidirectional RRT utilizing a 2% goal-biased sampling strategy. While demonstrating extreme speed in open spaces, it is susceptible to local minima. Included to demonstrate algorithmic evolution.

## How to run
To execute the planner and visualize the generated 3-D trajectories, run:
```bash
python main.py
```
Note on Visualization (`matplotlib`):
Due to Matplotlib's 3D rendering and event loops blocking the main Python thread, running multiple environments sequentially may cause the graphical window to hang.
To view trajectories properly, please uncomment and run only one test function at a time inside the `if __name__ == "__main__":` block at the bottom of `main.py`:
```bash
if __name__ == "__main__":
    # Uncomment ONE test below at a time:
    test_maze()             
    # test_room()
```
Close the Matplotlib window after reviewing a trajectory before running the next test. 

