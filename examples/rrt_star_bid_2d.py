# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.rrt_star_bid import RRTStarBidirectional
from src.utilities.plotting import Plot

X_dimensions = [(0, 100), (0, 100)]  # dimensions of Configuration Space
# obstacles
Obstacles = [(20, 20, 40, 40), (20, 60, 40, 80), (60, 20, 80, 40), (60, 60, 80, 80)]
x_init = (0, 0)  # starting location
x_goal = (100, 100)  # goal location

Q = [(8, 128)]  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
rewire_count = 32  # optional, number of nearby branches to rewire
prc = 0.01  # probability of checking for a connection to goal

# create Configuration Space
X = ConfigurationSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRTStarBidirectional(X, Q, max_samples, r, prc, rewire_count)
path, E_a, E_b = rrt.rrt_star_bidirectional(x_init, x_goal)

# plot
plot = Plot("rrt_star_bid_2d")
plot.plot_tree(X, [E_a, E_b])
plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
