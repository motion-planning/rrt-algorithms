# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import copy
import random
import uuid

from rtree import index

from src.a_star.a_star import a_star_search
from src.a_star.a_star import reconstruct_path
from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.primitive_procedures import can_connect_to_goal
from src.rrt.primitive_procedures import connect_to_goal
from src.rrt.primitive_procedures import steer
from src.utilities.conversion import convert_edge_set_to_dict
from src.utilities.geometry import distance_between_points


def rrt_star(X: ConfigurationSpace, x_init: tuple, max_samples: int, Q: list, r: float,
             x_goal: tuple, rewire_count: int = None, early_exit: bool = True, prc: float = 0.01) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    Based on algorithm found in: Incremental Sampling-based Algorithms for Optimal Motion Planning
    http://roboticsproceedings.org/rss06/p34.pdf
    :param X: Configuration Space
    :param x_init: initial location
    :param max_samples: max number of samples to take
    :param Q: length of new edges added to tree
    :param r: resolution of points to sample along edge when checking for collisions
    :param x_goal: goal location
    :param rewire_count: number of nearby branches to rewire
    :param early_exit: if allowed to check for connection to goal before generating all max_samples
    :param prc: probability of checking whether can connect to goal
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    custom_rewire = False if rewire_count is None else True
    p = index.Property()
    p.dimension = X.dimensions
    V_rtree = index.Index(interleaved=True, properties=p)
    V_rtree.insert(uuid.uuid4(), x_init + x_init, x_init)
    V_count = 1
    E = set()
    P = {x_init: None}

    samples_taken = 0

    while True:
        for q in Q:
            for i in range(q[1]):
                x_rand = X.sample_free()
                x_nearest = list(V_rtree.nearest(x_rand, num_results=1, objects="raw"))[0]
                x_new = steer(X, x_nearest, x_rand, q[0])

                if V_rtree.count(x_new) == 0 and X.collision_free(x_nearest, x_new, r):
                    rewire_count = V_count if not custom_rewire else rewire_count
                    X_near = list(V_rtree.nearest(x_new, num_results=rewire_count, objects="raw"))
                    V_rtree.insert(uuid.uuid4(), x_new + x_new, x_new)
                    V_count += 1
                    x_min = copy.deepcopy(x_nearest)
                    c_min = path_cost(P, x_init, x_nearest) + c(x_nearest, x_new)

                    for x_near in X_near:  # connect along a min-cost path
                        if path_cost(P, x_init, x_near) + c(x_near, x_new) < c_min and X.collision_free(x_near, x_new,
                                                                                                        r):
                            x_min = copy.deepcopy(x_near)
                            c_min = path_cost(P, x_init, x_near) + c(x_near, x_new)

                    E.add((x_min, x_new))
                    P[x_new] = x_min

                    for x_near in X_near:  # rewire tree
                        if path_cost(P, x_init, x_new) + c(x_new, x_near) < path_cost(P, x_init, x_near) and \
                                X.collision_free(x_new, x_near, r):
                            x_parent = P[x_near]

                            E.remove((x_parent, x_near))
                            P.pop(x_near)
                            E.add((x_new, x_near))
                            P[x_near] = x_new

                samples_taken += 1

                if early_exit and random.random() < prc:
                    print("Checking if can connect to goal at", str(samples_taken), "samples")
                    if can_connect_to_goal(X, V_rtree, x_goal, Q, r):
                        print("Can connect to goal")
                        E = connect_to_goal(V_rtree, E, x_goal)

                        return True, E

                if samples_taken >= max_samples:
                    if can_connect_to_goal(X, V_rtree, x_goal, Q, r):
                        print("Can connect to goal")
                        E = connect_to_goal(V_rtree, E, x_goal)

                        return True, E
                    else:
                        print("Could not connect to goal")

                    return False, E


def rrt_star_tree_path(X: ConfigurationSpace, x_init: tuple, max_samples: int, Q: list, r: float,
                       x_goal: tuple, rewire_count: int = None) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    :param X: Configuration Space
    :param x_init: initial location
    :param max_samples: max number of samples to take
    :param Q: length of new edges added to tree
    :param r: resolution of points to sample along edge when checking for collisions
    :param x_goal: goal location
    :param rewire_count: number of nearby branches to rewire
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    connected, E = rrt_star(X, x_init, max_samples, Q, r, x_goal, rewire_count)
    if not connected:
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
