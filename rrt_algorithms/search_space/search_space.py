# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import numpy as np
from rtree import index

from rrt_algorithms.utilities.geometry import es_points_along_line
from rrt_algorithms.utilities.obstacle_generation import obstacle_generator

class SearchSpace:
    def __init__(self, dimension_lengths, obstacles=None):
        self.dimensions = len(dimension_lengths)
        self.dimension_lengths = dimension_lengths
        self.obstacles = obstacles if obstacles is not None else []

    def obstacle_free(self, point):
        """
        Check if the point is free from obstacles.
        :param point: Tuple (x, y, z)
        :return: True if the point is free of obstacles, False otherwise.
        """
        for obstacle in self.obstacles:
            x_center, y_center, z_min, z_max, radius = obstacle
            
            # Check if the point is within the cylinder's height bounds
            if z_min <= point[2] <= z_max:
                # Check if the point is within the radius of the cylinder
                distance_to_axis = np.sqrt((point[0] - x_center) ** 2 + (point[1] - y_center) ** 2)
                if distance_to_axis <= radius:
                    return False  # The point is inside the obstacle
        return True  # The point is free of obstacles

    def collision_free(self, x_a, x_b, r):
        """
        Check if a line segment between x_a and x_b is free from obstacles.
        :param x_a: Starting point of the line segment.
        :param x_b: Ending point of the line segment.
        :param r: Resolution for sampling along the line.
        :return: True if the line segment does not intersect any obstacles, False otherwise.
        """
        # Generate points along the line segment between x_a and x_b
        points = self.es_points_along_line(x_a, x_b, r)
        for point in points:
            if not self.obstacle_free(point):
                return False  # Collision detected
        return True  # No collision

    

    def es_points_along_line(self, x_a, x_b, r):
        """
        Generate points along a line segment between x_a and x_b.
        :param x_a: Starting point of the line segment.
        :param x_b: Ending point of the line segment.
        :param r: Resolution for sampling points.
        :return: List of points along the line segment.
        """
        distance = np.linalg.norm(np.array(x_b) - np.array(x_a))
        num_points = int(distance / r) + 1
        return [tuple(np.array(x_a) + t * (np.array(x_b) - np.array(x_a))) for t in np.linspace(0, 1, num_points)]



    def sample(self):
        """
        Return a random location within the search space dimensions.
        :return: A tuple representing a random location within the search space.
        """
        return tuple(np.random.uniform(low, high) for low, high in self.dimension_lengths)

    def sample_free(self):
        """
        Sample a random location within the search space that is free from obstacles.
        :return: A tuple representing a random location within the search space that is obstacle-free.
        """
        while True:
            point = self.sample()
            if self.obstacle_free(point):
                return point


