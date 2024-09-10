# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np
import sys
sys.path.append('/home/dell/rrt-algorithms')

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space0 import SearchSpace
from rrt_algorithms.utilities.plotting0 import Plot



X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
# obstacles
Obstacles = np.array(
    [(10, 10, 10, 15, 15, 15),(20, 20, 20, 30, 30, 30),(30, 30, 30, 35, 35, 35), (1, 2, 6, 4, 4, 8), (20, 60, 20, 25, 65, 25), (60, 60, 20, 80, 80, 40),
     (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])
x_init = (0, 0, 0)  # starting location
x_goal = (8, 50, 50)  # goal location

q = 8  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRT(X, q, x_init, x_goal, max_samples, r, prc)
path = rrt.rrt_search()

# plot
plot = Plot("rrt_3d")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
