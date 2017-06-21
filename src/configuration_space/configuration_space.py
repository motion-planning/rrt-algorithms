# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import random


class ConfigurationSpace(object):
    def __init__(self, dimension_lengths: list, O: list):
        """
        Initialize Configuration Space
        :param dimension_lengths: range of each dimension
        :param O: obstacles
        """
        self.dimensions = len(dimension_lengths)  # number of dimensions
        self.dimension_lengths = dimension_lengths  # length of each dimension
        self.O = O  # obstacles

    def obstacle_free(self, x: tuple) -> bool:
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        for O_i in self.O:  # check each obstacle
            # check if point resides within range of each side of obstacle
            if all(i <= j <= k for i, j, k in zip(O_i[:self.dimensions], x, O_i[self.dimensions:])):
                return False

        return True

    def sample_free(self) -> tuple:
        """
        Sample a location within X_free
        :return: random location within X_free
        """
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start: tuple, end: tuple) -> bool:
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        for O_i in self.O:  # check each obstacle
            # check if line intersects range of each side of obstacle
            # min and max for line endpoints are done to allow for lines that go "backwards"
            if all(max(min(s_i, e_i), o_s) <= min(max(s_i, e_i), o_e) for s_i, o_s, e_i, o_e in
                   zip(start, O_i[:self.dimensions], end, O_i[self.dimensions:])):
                return False

        return True

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
