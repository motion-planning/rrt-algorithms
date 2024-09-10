import numpy as np
from rtree import index
import sys
sys.path.append('/home/dell/rrt-algorithms')

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.plotting import Plot
from rrt_algorithms.dwa_algorithm.DWA import DWA  # Assuming you have a DWA implementation

# Define the search space dimensions and obstacles
X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])
obstacles = [
    (10, 82, 0, 85, 15),  
    (80, 20, 0, 60, 19),   
    (10, 30, 0, 90, 15),   
    (80, 80, 0, 70, 22), 
    (50, 50, 0, 50, 22),
]

# Initial and goal positions
x_init = (-2, 0, 0)
x_intermediate = (50, 50, 100)
x_second_goal = (70, 20, 100)
x_final_goal = (80, 80, 100)

# RRT parameters
q = 30
r = 1
max_samples = 1024
prc = 0.1

# DWA parameters
dwa_params = {
    'max_speed': 1.0,
    'min_speed': 0.0,
    'max_yaw_rate': 40.0 * np.pi / 180.0,
    'max_accel': 0.2,
    'v_reso': 0.01,
    'yaw_rate_reso': 0.1 * np.pi / 180.0,
    'dt': 0.1,
    'predict_time': 3.0,
    'to_goal_cost_gain': 1.0,
    'speed_cost_gain': 1.0,
    'obstacle_cost_gain': 1.0,
}

# Create the search space
X = SearchSpace(X_dimensions, obstacles)

# RRT pathfinding to first intermediate goal
print("RRT to first intermediate goal...")
rrt = RRT(X, q, x_init, x_intermediate, max_samples, r, prc)
path1 = rrt.rrt_search()

if path1 is not None:
    print("RRT to second intermediate goal...")
    rrt = RRT(X, q, x_intermediate, x_second_goal, max_samples, r, prc)
    path2 = rrt.rrt_search()

    if path2 is not None:
        print("RRT to final goal...")
        rrt = RRT(X, q, x_second_goal, x_final_goal, max_samples, r, prc)
        path3 = rrt.rrt_search()

        # Combine all paths
        if path3 is not None:
            path = path1 + path2[1:] + path3[1:]
        else:
            path = path1 + path2[1:]
    else:
        path = path1
else:
    path = None

# Apply DWA for local optimization along the RRT path
if path is not None:
    dwa = DWA(dwa_params)
    optimized_path = []

    for i in range(len(path) - 1):
        start_point = path[i] + (0.0, 0.0)  # Initialize v and w to 0, using tuple
        end_point = path[i + 1]

        local_path = dwa.plan(start_point, end_point, X, obstacles)
        optimized_path.extend(local_path)

# Plotting
plot = Plot("rrt_dwa_3d")
plot.plot_tree(X, rrt.trees)
if optimized_path is not None:
    plot.plot_path(X, optimized_path)
plot.plot_obstacles(X, obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_intermediate, color="pink")
plot.plot_goal(X, x_second_goal, color="blue")
plot.plot_goal(X, x_final_goal, color="green")
plot.draw(auto_open=True)
