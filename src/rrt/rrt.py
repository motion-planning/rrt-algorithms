# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
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


def rrt(X: ConfigurationSpace, x_init: tuple, max_samples: int, Q: list, r: float,
        x_goal: tuple, early_exit: bool = True, prc: float = 0.01) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    :param X: Configuration Space
    :param x_init: initial location
    :param max_samples: max number of samples before timing out
    :param Q: length of new edges added to tree
    :param r: resolution of points to sample along edge when checking for collisions
    :param x_goal: goal location
    :param early_exit: if allowed to check for connection to goal before generating all max_samples
    :param prc: probability of checking whether can connect to goal
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    p = index.Property()
    p.dimension = X.dimensions
    V_rtree = index.Index(interleaved=True, properties=p)
    V_rtree.insert(uuid.uuid4(), x_init + x_init, x_init)
    V_count = 1
    E = set()
    samples_taken = 0

    while True:
        for q in Q:
            for i in range(q[1]):
                x_rand = X.sample_free()
                x_nearest = list(V_rtree.nearest(x_rand, num_results=1, objects="raw"))[0]
                x_new = steer(X, x_nearest, x_rand, q[0])

                if V_rtree.count(x_new) == 0 and X.collision_free(x_nearest, x_new, r):
                    V_rtree.insert(uuid.uuid4(), x_new + x_new, x_new)
                    V_count += 1
                    E.add((x_nearest, x_new))

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


def rrt_tree_path(X: ConfigurationSpace, x_init: tuple, max_samples: int, Q: list, r: float,
                  x_goal: tuple) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    :param X: Configuration Space
    :param x_init: initial location
    :param max_samples: max number of samples before timing out
    :param Q: length of new edges added to tree
    :param r: resolution of points to sample along edge when checking for collisions
    :param x_goal: goal location
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    connected, E = rrt(X, x_init, max_samples, Q, r, x_goal)
    if not connected:
        return E, []
    else:
        g = convert_edge_set_to_dict(E)
        came_from, cost_so_far = a_star_search(g, x_init, x_goal)
        path = reconstruct_path(came_from, x_init, x_goal)

        return E, path
