# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.rrt_search import RRTSearch
from src.utilities.plotting import Plot

X_dimensions = [(0, 100), (0, 100), (0, 100)]  # dimensions of Configuration Space
# obstacles
Obstacles = [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
             (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)]
x_init = (0, 0, 0)  # starting location
x_goal = (100, 100, 100)  # goal location

Q = [(8, 128)]  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
prc = 0.01  # probability of checking for a connection to goal

# create Configuration Space
X = ConfigurationSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRTSearch(X, Q, max_samples, r, prc)
path, E = rrt.rrt_search(x_init, x_goal)

# plot
plot = Plot("rrt_3d")
plot.plot_tree(X, [E])
plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
