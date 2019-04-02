# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import math
from itertools import tee


def dist_between_points(a, b):
    """
    Return the Euclidean distance between two points
    :param a: first point
    :param b: second point
    :return: Euclidean distance between a and b
    """
    distance = sum(map(lambda a_b: (a_b[0] - a_b[1]) ** 2, zip(a, b)))

    return math.sqrt(distance)


def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = tee(iterable)
    next(b, None)

    return zip(a, b)
