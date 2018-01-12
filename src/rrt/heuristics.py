# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from src.utilities.geometry import distance_between_points


def cost_to_go(a: tuple, b: tuple) -> float:
    """
    :param a: current location
    :param b: next location
    :return: estimated segment_cost-to-go from a to b
    """
    return distance_between_points(a, b)


def path_cost(P: dict, x_init: tuple, x: tuple) -> float:
    """
    Cost of the unique path from x_init to x
    :param P: parents of children, in form of child: parent
    :param x_init: initial location
    :param x: goal location
    :return: segment_cost of unique path from x_init to x
    """
    cost = 0
    while not x == x_init:
        p = P[x]
        cost += distance_between_points(x, p)
        x = p

    return cost


def segment_cost(a: tuple, b: tuple) -> float:
    """
    Cost function of the line between x_near and x_new
    :param a: start of line
    :param b: end of line
    :return: segment_cost function between a and b
    """
    dist = distance_between_points(a, b)

    return dist
