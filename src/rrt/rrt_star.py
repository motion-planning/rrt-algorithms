# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import random
import uuid
from operator import itemgetter

from rtree import index

from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.heuristics import segment_cost, path_cost, cost_to_go
from src.rrt.primitive_procedures import can_connect_to_goal, reconstruct_path
from src.rrt.primitive_procedures import connect_to_goal
from src.rrt.primitive_procedures import steer


def rrt_star(X: ConfigurationSpace, x_init: tuple, max_samples: int, Q: list, r: float,
             x_goal: tuple, rewire_count: int = None, prc: float = None) -> (set, dict):
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
    :param prc: probability of checking whether can connect to goal
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    custom_rewire = False if rewire_count is None else True
    p = index.Property()
    p.dimension = X.dimensions
    V_rtree = index.Index(interleaved=True, properties=p)
    V_rtree.insert(uuid.uuid4(), x_init + x_init, x_init)
    V_count = 1
    P = {x_init: None}

    c_best = float('inf')

    samples_taken = 0

    while True:
        for q in Q:
            for i in range(q[1]):
                x_rand = X.sample_free()
                samples_taken += 1
                x_nearest = list(V_rtree.nearest(x_rand, num_results=1, objects="raw"))[0]
                x_new = steer(X, x_nearest, x_rand, q[0])
                if not X.obstacle_free(x_new) or not V_rtree.count(x_new) == 0:
                    continue

                rewire_count = V_count if not custom_rewire else min(V_count, rewire_count)
                X_near = list(V_rtree.nearest(x_new, num_results=rewire_count, objects="raw"))
                L_near = [(x_near, path_cost(P, x_init, x_near) + segment_cost(x_near, x_new)) for x_near in X_near]
                # noinspection PyTypeChecker
                L_near.sort(key=itemgetter(1))

                for x_near, c_near in L_near:
                    if c_near + cost_to_go(x_near, x_goal) < c_best:
                        if X.collision_free(x_near, x_new, r):
                            V_rtree.insert(uuid.uuid4(), x_new + x_new, x_new)
                            V_count += 1
                            P[x_new] = x_near

                            break

                if x_new in P:
                    for x_near, c_near in L_near:
                        if path_cost(P, x_init, x_new) + c_near < path_cost(P, x_init, x_near):
                            if X.collision_free(x_near, x_new, r):
                                P.pop(x_near)
                                P[x_near] = x_new

                if prc and random.random() < prc:
                    print("Checking if can connect to goal at", str(samples_taken), "samples")
                    if can_connect_to_goal(X, V_rtree, x_goal, Q, r):
                        print("Can connect to goal")
                        P = connect_to_goal(V_rtree, P, x_goal)

                        return True, P

                if samples_taken >= max_samples:
                    if can_connect_to_goal(X, V_rtree, x_goal, Q, r):
                        print("Can connect to goal")
                        P = connect_to_goal(V_rtree, P, x_goal)

                        return True, P
                    else:
                        print("Could not connect to goal")

                    return False, P


def rrt_star_tree_path(X: ConfigurationSpace, x_init: tuple, max_samples: int, Q: list, r: float, x_goal: tuple,
                       rewire_count: int = None, prc: float = None) -> (set, dict):
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
    :param prc: probability of checking whether can connect to goal
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    connected, P = rrt_star(X, x_init, max_samples, Q, r, x_goal, rewire_count, prc)
    if not connected:
        return P, []
    else:
        path = reconstruct_path(P, x_init, x_goal)

        return P, path
