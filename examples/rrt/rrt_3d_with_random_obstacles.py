# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.obstacle_generation import generate_random_obstacles
from rrt_algorithms.utilities.plotting import Plot

X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
x_init = (0, 0, 0)  # starting location
x_goal = (100, 100, 100)  # goal location

q = 8  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
prc = 0.1  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions)
n = 50
Obstacles = generate_random_obstacles(X, x_init, x_goal, n)
# create rrt_search
rrt = RRT(X, q, x_init, x_goal, max_samples, r, prc)
path = rrt.rrt_search()

# plot
plot = Plot("rrt_3d_with_random_obstacles")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
