# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.rrt import rrt_tree_path
from src.utilities.plotting import Plot

q = 10  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
# obstacles
Obstacles = [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
             (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)]
x_init = (0, 0, 0)  # starting location
x_goal = (100, 100, 100)  # goal location
n = 100  # number of samples to take each iteration
max_samples = 5000  # max number of samples to take before timing out
X_dimensions = [(0, 100), (0, 100), (0, 100)]  # dimensions of Configuration Space

# create Configuration Space
X = ConfigurationSpace(X_dimensions, Obstacles)

# create rrt
E, path = rrt_tree_path(X, x_init, n, max_samples, q, r, x_goal)

# plot
plot = Plot("example_3d_rrt")
plot.plot_tree(X, E)
plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw()
