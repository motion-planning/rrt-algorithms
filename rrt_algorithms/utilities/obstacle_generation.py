from __future__ import annotations

import random
import uuid

import numpy as np

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rrt_algorithms.search_space.search_space import SearchSpace



def generate_random_obstacles(X: SearchSpace, start, end, n):
    """
    Generates n random obstacles without disrupting world connectivity.
    It also respects start and end points so that they don't lie inside of an obstacle.
    """
    # Note: Current implementation only supports hyperrectangles.
    i = 0
    obstacles = []
    while i < n:
        center = np.empty(len(X.dimension_lengths), float)
        scollision = True
        fcollision = True
        edge_lengths = []
        for j in range(X.dimensions):
            # None of the sides of a hyperrectangle can be higher than 0.1 of the total span
            # in that particular X.dimensions
            max_edge_length = (X.dimension_lengths[j][1] - X.dimension_lengths[j][0]) / 10.0
            # None of the sides of a hyperrectangle can be higher than 0.01 of the total span
            # in that particular X.dimensions
            min_edge_length = (X.dimension_lengths[j][1] - X.dimension_lengths[j][0]) / 100.0
            edge_length = random.uniform(min_edge_length, max_edge_length)
            center[j] = random.uniform(X.dimension_lengths[j][0] + edge_length,
                                       X.dimension_lengths[j][1] - edge_length)
            edge_lengths.append(edge_length)

            if abs(start[j] - center[j]) > edge_length:
                scollision = False
            if abs(end[j] - center[j]) > edge_length:
                fcollision = False

        # Check if any part of the obstacle is inside of another obstacle.
        min_corner = np.empty(X.dimensions, float)
        max_corner = np.empty(X.dimensions, float)
        for j in range(X.dimensions):
            min_corner[j] = center[j] - edge_lengths[j]
            max_corner[j] = center[j] + edge_lengths[j]
        obstacle = np.append(min_corner, max_corner)
        # Check newly generated obstacle intersects any former ones. Also respect start and end points
        if len(list(X.obs.intersection(obstacle))) > 0 or scollision or fcollision:
            continue
        i += 1
        obstacles.append(obstacle)
        X.obs.insert(uuid.uuid4().int, tuple(obstacle), tuple(obstacle))

    return obstacles


def obstacle_generator(obstacles):
    """
    Add obstacles to r-tree
    :param obstacles: list of obstacles
    """
    for obstacle in obstacles:
        yield (uuid.uuid4().int, obstacle, obstacle)
