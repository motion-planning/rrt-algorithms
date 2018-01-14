# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from operator import add

from rtree import index

from src.configuration_space.configuration_space import ConfigurationSpace
from src.utilities.geometry import distance_between_points


def steer(X: ConfigurationSpace, start: tuple, goal: tuple, distance: float) -> tuple:
    """
    Return a point in the direction of the goal, that is distance away from start
    :param X: Configuration Space
    :param start: start location
    :param goal: goal location
    :param distance: distance away from start
    :return: point in the direction of the goal, distance away from start
    """
    ab = []  # difference between start and goal
    for start_i, goal_i in zip(start, goal):
        ab.append(goal_i - start_i)

    ab = tuple(ab)
    zero_vector = tuple([0] * len(ab))

    ba_length = distance_between_points(zero_vector, ab)  # get length of vector ab
    unit_vector = [i / ba_length for i in ab]  # normalize
    scaled_vector = [i * distance for i in unit_vector]  # scale vector to desired length

    steered_point = list(map(add, start, scaled_vector))  # add scaled vector to starting location for final point

    # if point is out-of-bounds, set to bound
    for dim, dim_range in enumerate(X.dimension_lengths):
        if steered_point[dim] < dim_range[0]:
            steered_point[dim] = dim_range[0]
        elif steered_point[dim] > dim_range[1]:
            steered_point[dim] = dim_range[1]

    steered_point = tuple(steered_point)

    return steered_point


def can_connect_to_goal(X: ConfigurationSpace, V_rtree: index, x_goal: tuple, Q: list, r: float) -> bool:
    """
    Check if the goal can be connected to the graph
    :param X: Configuration Space
    :param V_rtree: rtree of all Vertices
    :param x_goal: goal location we want to add
    :param Q: length of edges to add
    :param r: resolution of points to sample along edge when checking for collisions
    :return: True if can be added, False otherwise
    """
    x_nearest = list(V_rtree.nearest(x_goal, num_results=1, objects="raw"))[0]
    distance = distance_between_points(x_nearest, x_goal)
    if distance > max(Q, key=lambda x: x[1])[1]:  # check if close enough
        return False

    if X.collision_free(x_nearest, x_goal, r):  # check if obstacle-free
        return True

    return False


def can_connect_to_tree(X: ConfigurationSpace, V_rtree: index, x_goal: tuple, Q: list, r: float) -> bool:
    """
    Check if the goal can be connected to the graph
    :param X: Configuration Space
    :param V_rtree: rtree of all Vertices
    :param x_goal: goal location we want to add
    :param Q: length of edges to add
    :param r: resolution of points to sample along edge when checking for collisions
    :return: True if can be added, False otherwise
    """
    x_nearest = list(V_rtree.nearest(x_goal, num_results=1, objects="raw"))[0]
    distance = distance_between_points(x_nearest, x_goal)
    if distance > max(Q, key=lambda x: x[1])[1]:  # check if close enough
        return False

    if X.collision_free(x_nearest, x_goal, r):  # check if obstacle-free
        return True

    return False


def connect_to_goal(V_rtree: index, P: dict, x_goal: tuple) -> dict:
    """
    Connect x_goal to graph (does not check if this should be possible, for that use: can_connect_to_goal)
    :param V_rtree: rtree of all Vertices
    :param P: dict of all Edges in form: P[child] = parent
    :param x_goal: goal location to add
    :return: updated P
    """
    x_nearest = list(V_rtree.nearest(x_goal, num_results=1, objects="raw"))[0]
    P[x_goal] = x_nearest

    return P


def reconstruct_path(E: dict, x_init: tuple, x_goal: tuple) -> list:
    """
    Reconstruct path from start to goal
    :param E: dict of all Edges in form: E[child] = parent
    :param x_init: starting location
    :param x_goal: goal location
    :return: sequence of vertices from start to goal
    """
    path = [x_goal]
    current = x_goal
    if x_init == x_goal:
        return path
    while not E[current] == x_init:
        path.append(E[current])
        current = E[current]
    path.append(x_init)
    path.reverse()

    return path
