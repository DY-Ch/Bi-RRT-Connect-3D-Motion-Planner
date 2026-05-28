# Bi-RRT-Connect-3D-Motion-Planner
A Bi-Directional RRT-Connect motion planner for continuous 3D environments, featuring sub-0.033s dynamic fast-replanning and vectorized AABB collision checking.



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

