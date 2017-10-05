# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import math
import random
import uuid

import numpy as np
from rtree import index

from src.utilities.geometry import distance_between_points


class ConfigurationSpace(object):
    def __init__(self, dimension_lengths: list, O: list):
        """
        Initialize Configuration Space
        :param dimension_lengths: range of each dimension
        :param O: obstacles
        """
        self.dimensions = len(dimension_lengths)  # number of dimensions
        self.dimension_lengths = dimension_lengths  # length of each dimension
        p = index.Property()
        p.dimension = self.dimensions
        self.idx = index.Index(interleaved=True, properties=p)  # r-tree representation of obstacles
        self.insert_obstacles(O)

    def insert_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.idx.insert(uuid.uuid4(), obstacle)

    def obstacle_free(self, x: tuple) -> bool:
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        return len(list(self.idx.intersection(x))) == 0

    def sample_free(self) -> tuple:
        """
        Sample a location within X_free
        :return: random location within X_free
        """
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start: tuple, end: tuple, r: float) -> bool:
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        n_points = int(math.ceil(distance_between_points(start, end) / r))
        # prev = 0
        # for i in range(11, 0, -2):
        #     scaled_n_points = max(3, int(math.ceil(n_points / i)))
        #     if scaled_n_points == prev:
        #         continue
        #
        #     prev = scaled_n_points

        dim_linspaces = [np.linspace(s_i, e_i, n_points) for s_i, e_i in zip(start, end)]
        points = [point for point in zip(*dim_linspaces)]

        if all(self.obstacle_free(point) for point in points):
            return True

        return False

    def sample(self) -> tuple:
        """
        Return a random location within X
        :return: random location within X (not necessarily X_free)
        """
        x = []
        for dimension in range(len(self.dimension_lengths)):
            x.append(random.uniform(self.dimension_lengths[dimension][0], self.dimension_lengths[dimension][1]))

        x = tuple(x)

        return x
