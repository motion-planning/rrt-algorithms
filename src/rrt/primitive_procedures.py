import operator
from operator import add

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


def can_connect_to_goal(X: ConfigurationSpace, V: set, x_goal: tuple, q: float) -> bool:
    """
    Check if the goal can be connected to the graph
    :param X: Configuration Space
    :param V: set of all Vertices
    :param x_goal: goal location we want to add
    :param q: length of edges to add
    :return: True if can be added, False otherwise
    """
    x_nearest = nearest_vertices(V, x_goal)
    distance = distance_between_points(x_nearest[0], x_goal)
    if distance > q:  # check if close enough
        return False

    if X.obstacle_free(x_nearest[0], x_goal):  # check if obstacle-free
        return True

    return False


def connect_to_goal(V: set, E: set, x_goal: tuple) -> set:
    """
    Connect x_goal to graph (does not check if this should be possible, for that use: can_connect_to_goal)
    :param V: set of all Vertices
    :param E: set of all Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    :param x_goal: goal location to add
    :return: updated E
    """
    x_nearest = nearest_vertices(V, x_goal)[0]
    E.add((x_nearest, x_goal))

    return E


def nearest_vertices(V: set, x: tuple, n: int = 1) -> list:
    """
    Return nearest Vertex to x
    :param V: set of all Vertices
    :param x: vertex we want to get near to
    :param n: number of samples to draw (defaults to 1)
    :return: nearest Vertex
    """
    vertex_distances = []
    for v in V:  # distances to all vertices
        dist = distance_between_points(v, x)
        vertex_distances.append((dist, v))

    vertex_distances.sort(key=operator.itemgetter(0))  # sort by shortest distance
    vertex_distances = vertex_distances[:n]  # n closest vertices

    nearest = []
    for v in vertex_distances:  # remove distance from tuple and only keep vertices
        nearest.append(v[1])

    return nearest
