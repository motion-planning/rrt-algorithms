# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#the RRT and DWA algorithms function with the optimized collision checking using Bounding Volume Hierarchies (BVH).
#BVH Implementation: We use an R-tree (a type of BVH) to store axis-aligned bounding boxes (AABB) for each obstacle. This allows us to quickly identify which obstacles might intersect with a given point or path segment.
#Efficient Collision Checking: Instead of checking every obstacle for collision, we first use the BVH to narrow down the list to only those obstacles whose bounding boxes intersect with the point or path segment in question.
#Integration with DWA: The code remains the same in terms of how paths are generated and optimized, but now the collision checking is more efficient, leading to faster overall performance.
#All obstacles (both random and predefined) need to be properly registered in the SearchSpace so that the collision checking functions (obstacle_free and collision_free) are aware of all obstacles.
# the script avoids both static and dynamic obstacles, it starts from orange dot and its final goal is green.
from __future__ import annotations

import random
import uuid

import numpy as np

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rrt_algorithms.search_space.search_space2 import SearchSpace


def generate_random_cylindrical_obstacles(X: SearchSpace, start: Tuple[float, float, float], end: Tuple[float, float, float], n: int) -> List[Tuple[float, float, float, float, float]]:
    """
    Generates n random cylindrical obstacles without disrupting world connectivity.
    It also respects start and end points so that they don't lie inside an obstacle.
    """
    obstacles = []
    i = 0
    while i < n:
        scollision = True
        fcollision = True

        x_center = random.uniform(X.dimension_lengths[0][0], X.dimension_lengths[0][1])
        y_center = random.uniform(X.dimension_lengths[1][0], X.dimension_lengths[1][1])
        z_min = random.uniform(X.dimension_lengths[2][0], X.dimension_lengths[2][1] - 10)  # Ensure some height
        z_max = z_min + random.uniform(5, 15)  # Give a random height between 5 and 15
        radius = random.uniform(2, 10)  # Random radius between 2 and 10

        if np.linalg.norm([start[0] - x_center, start[1] - y_center]) > radius:
            scollision = False
        if np.linalg.norm([end[0] - x_center, end[1] - y_center]) > radius:
            fcollision = False

        obstacle = (x_center, y_center, z_min, z_max, radius)

        if scollision or fcollision:
            continue

        obstacles.append(obstacle)
        X.obstacles.append(obstacle)  # Add to the SearchSpace's obstacles list
        i += 1

    # Rebuild the BVH with the new obstacles
    X.build_bvh()

    return obstacles


def obstacle_generator(obstacles):
    """
    Add obstacles to r-tree
    :param obstacles: list of obstacles
    """
    for obstacle in obstacles:
        yield (uuid.uuid4().int, obstacle, obstacle)
