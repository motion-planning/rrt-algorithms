# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from src.a_star.a_star import a_star_search
from src.a_star.a_star import reconstruct_path
from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.primitive_procedures import can_connect_to_goal
from src.rrt.primitive_procedures import connect_to_goal
from src.rrt.primitive_procedures import steer, nearest_vertices
from src.utilities.conversion import convert_edge_set_to_dict


def rrt_until_connect(X: ConfigurationSpace, x_init: tuple, n: int, max_samples: int, q: float,
                      x_goal: tuple) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    :param X: Configuration Space
    :param x_init: initial location
    :param n: number of samples to take each iteration
    :param max_samples: max number of samples before timing out
    :param q: length of new edges added to tree
    :param x_goal: goal location
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    V = {x_init}
    E = set()
    samples_taken = 0

    while True:
        if can_connect_to_goal(X, V, x_goal, q):
            print("Testing: Can connect to goal")
            E = connect_to_goal(V, E, x_goal)
            break

        print("Can't connect to goal yet")
        print("Expanding tree")
        for i in range(n):
            x_rand = X.sample_free()
            x_nearest = nearest_vertices(V, x_rand)[0]
            x_new = steer(X, x_nearest, x_rand, q)

            if X.collision_free(x_nearest, x_new):
                V.add(x_new)
                E.add((x_nearest, x_new))

        samples_taken += n
        if samples_taken > max_samples:
            print("Could not connect to goal")
            V, E = None, None
            break

        print("Finished expanding tree")

    return V, E


def rrt_tree_path(X: ConfigurationSpace, x_init: tuple, n: int, max_samples: int, q: float,
                  x_goal: tuple) -> (set, dict):
    """
    Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
    https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
    :param X: Configuration Space
    :param x_init: initial location
    :param n: number of samples to take between testing for goal connection
    :param max_samples: max number of samples before timing out
    :param q: length of new edges added to tree
    :param x_goal: goal location
    :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
    """
    V, E = rrt_until_connect(X, x_init, n, max_samples, q, x_goal)
    if V is None and E is None:
        return []
    else:
        g = convert_edge_set_to_dict(E)
        came_from, cost_so_far = a_star_search(g, x_init, x_goal)
        path = reconstruct_path(came_from, x_init, x_goal)

        return E, path
