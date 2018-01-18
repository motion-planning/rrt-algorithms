# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import math


def distance_between_points(a, b):
    """
    Return the Euclidean distance between two points
    :param a: first point
    :param b: second point
    :return: Euclidean distance between a and b
    """
    distance = 0
    for a_i, b_i in zip(a, b):
        distance += (a_i - b_i) ** 2

    return math.sqrt(distance)
