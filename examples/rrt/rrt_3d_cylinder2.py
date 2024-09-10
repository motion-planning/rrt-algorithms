import numpy as np
from rtree import index  # Import the index module from the rtree package
import sys
sys.path.append('/home/dell/rrt-algorithms')

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.plotting import Plot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Define the search space dimensions
X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])

# Define cylindrical obstacles
obstacles = [
    (10, 82, 0, 85, 15),  
    (80, 20, 0, 60, 19),   
    (10, 30, 0, 90, 15),   
    (80, 80, 0, 70, 22), 
    (50, 50, 0, 50, 22),  # Cylinder at (50, 50) with height 15 along z-axis and radius 5
]

# Define initial and goal positions
x_init = (-2, 0, 0)  # Starting location
x_intermediate = (50, 50, 100)  # Intermediate goal location
x_goal = (80, 80, 100)  # Final goal location

# Define RRT parameters
q = 30  # Length of tree edges
r = 1  # Length of smallest edge to check for intersection with obstacles
max_samples = 1024  # Max number of samples to take before timing out
prc = 0.1  # Probability of checking for a connection to goal

# Create the search space
X = SearchSpace(X_dimensions, obstacles)

# First RRT search: from x_init to x_intermediate
print("Starting RRT search from initial point to intermediate goal...")
rrt = RRT(X, q, x_init, x_intermediate, max_samples, r, prc)
path1 = rrt.rrt_search()

# If a path is found to the intermediate goal, continue to the final goal
if path1 is not None:
    # Second RRT search: from x_intermediate to x_goal
    print("Starting RRT search from intermediate goal to final goal...")
    rrt = RRT(X, q, x_intermediate, x_goal, max_samples, r, prc)
    path2 = rrt.rrt_search()

    # Combine the two paths
    if path2 is not None:
        path = path1 + path2[1:]  # Combine paths, avoiding duplicate point at x_intermediate
    else:
        print("Could not find a path to the final goal.")
        path = path1  # Use only the first path
else:
    print("Could not find a path to the intermediate goal.")
    path = None

# Plotting setup
plot = Plot("rrt_3d")

# Plot the RRT tree, path, and cylindrical obstacles
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, obstacles)  # Plot the cylindrical obstacles
plot.plot_start(X, x_init)
plot.plot_goal(X, x_intermediate, color="pink")  # Plot the intermediate goal in pink
plot.plot_goal(X, x_goal, color="green")  # Plot the final goal in green
plot.draw(auto_open=True)

