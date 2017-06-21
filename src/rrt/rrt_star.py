# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import copy

from src.a_star.a_star import a_star_search
from src.a_star.a_star import reconstruct_path
from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.primitive_procedures import can_connect_to_goal
from src.rrt.primitive_procedures import connect_to_goal
from src.rrt.primitive_procedures import steer, nearest_vertices
from src.utilities.conversion import convert_edge_set_to_dict
from src.utilities.geometry import distance_between_points


def rrt_star_until_connect(X: ConfigurationSpace, x_init: tuple, n: int, max_samples: int, q: float,
                           x_goal: tuple) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    Based on algorithm found in: Incremental Sampling-based Algorithms for Optimal Motion Planning
    http://roboticsproceedings.org/rss06/p34.pdf
    :param X: Configuration Space
    :param x_init: initial location
    :param n: number of samples to take between iterations
    :param max_samples: max number of samples to take
    :param q: length of new edges added to tree
    :param x_goal: goal location
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    V = {x_init}
    E = set()
    P = {x_init: None}

    samples_taken = 0

    while True:
        if can_connect_to_goal(X, V, x_goal, q):
            print("Testing: Can connect to goal")
            E = connect_to_goal(V, E, x_goal)
            break

        print("Can't connect to goal yet")
        print("Expanding tree at " + str(samples_taken) + " samples")

        for i in range(n):
            x_rand = X.sample_free()
            x_nearest = nearest_vertices(V, x_rand)[0]
            x_new = steer(X, x_nearest, x_rand, q)

            if X.collision_free(x_nearest, x_new):
                X_near = nearest_vertices(V, x_new, len(V))
                V.add(x_new)
                x_min = copy.deepcopy(x_nearest)
                c_min = path_cost(P, x_init, x_nearest) + c(x_nearest, x_new)

                for x_near in X_near:  # connect along a min-cost path
                    if X.collision_free(x_near, x_new) and \
                                            path_cost(P, x_init, x_near) + c(x_near, x_new) < c_min:
                        x_min = copy.deepcopy(x_near)
                        c_min = path_cost(P, x_init, x_near) + c(x_near, x_new)

                E.add((x_min, x_new))
                P[x_new] = x_min

                for x_near in X_near:  # rewire tree
                    if X.collision_free(x_new, x_near) and \
                                            path_cost(P, x_init, x_new) + c(x_new, x_near) < path_cost(P, x_init,
                                                                                                       x_near):
                        x_parent = P[x_near]

                        E.remove((x_parent, x_near))
                        P.pop(x_near)
                        E.add((x_new, x_near))
                        P[x_near] = x_new

        samples_taken += n
        if samples_taken > max_samples:
            print("Could not connect to goal")
            V = None
            break

        print("Finished expanding tree")

    return V, E


def rrt_star_tree_path(X: ConfigurationSpace, x_init: tuple, n: int, max_samples: int, q: float, x_goal: tuple) -> (
        set, dict):
    """
    Create and return a Rapidly-exploring Random Tree
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    :param X: Configuration Space
    :param x_init: initial location
    :param n: number of samples to take between iterations
    :param max_samples: max number of samples to take
    :param q: length of new edges added to tree
    :param x_goal: goal location
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    V, E = rrt_star_until_connect(X, x_init, n, max_samples, q, x_goal)
    if V is None:
        return E, []
    else:
        g = convert_edge_set_to_dict(E)
        came_from, cost_so_far = a_star_search(g, x_init, x_goal)
        path = reconstruct_path(came_from, x_init, x_goal)

        return E, path


def path_cost(P: dict, x_init: tuple, x: tuple) -> float:
    """
    Cost of the unique path from x_init to x
    :param P: parents of children, in form of child: parent
    :param x_init: initial location
    :param x: goal location
    :return: cost of unique path from x_init to x
    """
    cost = 0
    while not x == x_init:
        p = P[x]
        cost += distance_between_points(x, p)
        x = p

    return cost


def c(a: tuple, b: tuple) -> float:
    """
    Cost function of the line between x_near and x_new
    :param a: start of line
    :param b: end of line
    :return: cost function between a and b
    """
    dist = distance_between_points(a, b)

    return dist
