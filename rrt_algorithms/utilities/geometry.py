# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from itertools import tee

import numpy as np


def dist_between_points(a, b):
    """
    Return the Euclidean distance between two points
    :param a: first point
    :param b: second point
    :return: Euclidean distance between a and b
    """
    distance = np.linalg.norm(np.array(b) - np.array(a))
    return distance


def pairwise(iterable):
    """
    Pairwise iteration over iterable
    :param iterable: iterable
    :return: s -> (s0,s1), (s1,s2), (s2, s3), ...
    """
    a, b = tee(iterable)
    next(b, None)
    return zip(a, b)


def es_points_along_line(start, end, r):
    """
    Equally-spaced points along a line defined by start, end, with resolution r
    :param start: starting point
    :param end: ending point
    :param r: maximum distance between points
    :return: yields points along line from start to end, separated by distance r
    """
    d = dist_between_points(start, end)
    n_points = int(np.ceil(d / r))
    if n_points > 1:
        step = d / (n_points - 1)
        for i in range(n_points):
            next_point = steer(start, end, i * step)
            yield next_point


def steer(start, goal, d):
    """
    Return a point in the direction of the goal, that is distance away from start
    :param start: start location
    :param goal: goal location
    :param d: distance away from start
    :return: point in the direction of the goal, distance away from start
    """
    start, end = np.array(start), np.array(goal)
    v = end - start
    u = v / (np.sqrt(np.sum(v ** 2)))
    steered_point = start + u * d
    return tuple(steered_point)
