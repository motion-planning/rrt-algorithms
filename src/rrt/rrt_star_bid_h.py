# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import random
import uuid
from operator import itemgetter

from rtree import index

from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.heuristics import segment_cost, path_cost, cost_to_go
from src.rrt.primitive_procedures import reconstruct_path
from src.rrt.primitive_procedures import steer
from src.rrt.rrt import RRT


class RRTStarBidirectionalHeuristic(RRT):
    def __init__(self, X: ConfigurationSpace, Q: list, max_samples: int, r: float, prc: float,
                 rewire_count: int = None):
        """
        Bidirectional RRT* Search
        :param X: Configuration Space
        :param Q: list of lengths of edges added to tree
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        :param rewire_count: number of nearby vertices to rewire
        """
        super().__init__(X, Q, max_samples, r, prc)
        self.rewire_count = rewire_count

    def rrt_star_bidirectional_heuristic(self, x_init: tuple, x_goal: tuple) -> (set, dict):
        """
        :param x_init: initial location
        :param x_goal: goal location
        :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
        """
        p = index.Property()
        p.dimension = self.X.dimensions
        # tree a
        V_a = index.Index(interleaved=True, properties=p)
        V_a.insert(uuid.uuid4(), x_init + x_init, x_init)
        V_a_count = 1
        E_a = {x_init: None}

        # tree b
        V_b = index.Index(interleaved=True, properties=p)
        V_b.insert(uuid.uuid4(), x_goal + x_goal, x_goal)
        V_b_count = 1
        E_b = {x_goal: None}

        c_best = float('inf')  # length of best solution thus far
        sigma_best = None  # best solution thus far

        samples_taken = 0

        while True:
            for q in self.Q:  # iterate over different edge lengths
                for i in range(q[1]):  # iterate over number of edges of given length to add
                    x_rand = self.X.sample_free()
                    samples_taken += 1
                    x_nearest = list(V_a.nearest(x_rand, num_results=1, objects="raw"))[0]
                    x_new = steer(self.X, x_nearest, x_rand, q[0])
                    if not self.X.obstacle_free(x_new) or not V_a.count(x_new) == 0:
                        continue

                    # get nearby vertices and cost-to-come
                    rewire_count = 1
                    if sigma_best is not None:
                        if self.rewire_count is None:
                            rewire_count = V_a_count
                        else:
                            rewire_count = min(V_a_count, self.rewire_count)
                    X_near = list(V_a.nearest(x_new, num_results=rewire_count, objects="raw"))
                    L_near = [(x_near, path_cost(E_a, x_init, x_near) +
                               segment_cost(x_near, x_new)) for x_near in X_near]
                    # noinspection PyTypeChecker
                    L_near.sort(key=itemgetter(1))

                    # check nearby vertices for total cost and connect shortest valid edge
                    for x_near, c_near in L_near:
                        if c_near + cost_to_go(x_near, x_goal) < c_best:
                            if self.X.collision_free(x_near, x_new, self.r):
                                V_a.insert(0, x_new + x_new, x_new)
                                V_a_count += 1
                                E_a[x_new] = x_near

                                break

                    # rewire tree
                    if x_new in E_a:
                        for x_near, c_near in L_near:
                            pc = path_cost(E_a, x_init, x_new) + c_near
                            if pc < path_cost(E_a, x_init, x_near):
                                if self.X.collision_free(x_near, x_new, self.r):
                                    E_a[x_near] = x_new

                        # nearby vertices from opposite tree and cost-to-come
                        rewire_count = 1
                        if sigma_best is not None:
                            if self.rewire_count is None:
                                rewire_count = V_b_count
                            else:
                                rewire_count = min(V_b_count, self.rewire_count)
                        X_near = list(V_b.nearest(x_new, num_results=rewire_count, objects="raw"))
                        L_near = [(x_near, path_cost(E_b, x_goal, x_near) +
                                   segment_cost(x_near, x_new)) for x_near in X_near]
                        # noinspection PyTypeChecker
                        L_near.sort(key=itemgetter(1))

                        # check nearby vertices for total cost and connect shortest valid edge
                        # this results in both trees being connected
                        for x_near, c_near in L_near:
                            c_tent = c_near + path_cost(E_a, x_init, x_new)
                            if c_tent < c_best:
                                if self.X.collision_free(x_near, x_new, self.r):
                                    V_b_count += 1
                                    E_b[x_new] = x_near
                                    c_best = c_tent
                                    sigma_a = reconstruct_path(E_a, x_init, x_new)
                                    sigma_b = reconstruct_path(E_b, x_goal, x_new)
                                    del sigma_b[-1]
                                    sigma_b.reverse()
                                    sigma_best = sigma_a + sigma_b

                                    break

                    # lazily attempt to contract random segment of best path
                    if sigma_best is not None and len(sigma_best) > 2:
                        a, b = 0, 0
                        while not abs(a - b) > 1:
                            a, b = random.sample(range(0, len(sigma_best)), 2)
                        v_a, v_b = sigma_best[a], sigma_best[b]

                        if self.X.collision_free(v_a, v_b, self.r):
                            pc_a_b = segment_cost(v_a, v_b)

                            # create new edge connecting vertices
                            if v_a in E_a and v_b in reconstruct_path(E_a, x_init, v_a):
                                E_a[v_a] = v_b
                            elif v_a in E_b and v_b in reconstruct_path(E_b, x_goal, v_a):
                                E_b[v_a] = v_b
                            elif v_b in E_a and v_a in reconstruct_path(E_a, x_init, v_b):
                                E_a[v_b] = v_a
                            elif v_b in E_b and v_a in reconstruct_path(E_b, x_goal, v_b):
                                E_b[v_b] = v_a
                            elif v_a in E_a:
                                E_a[v_b] = v_a
                            else:
                                E_b[v_b] = v_a

                            # update best path
                            sigma_best = sigma_best[:min(a, b) + 1] + sigma_best[max(a, b):]

                    if self.prc and random.random() < self.prc:  # probabilistically check if solution found
                        print("Checking if can connect to goal at", str(samples_taken), "samples")
                        if sigma_best is not None:
                            print("Can connect to goal")

                            return sigma_best, E_a, E_b

                    if samples_taken >= self.max_samples:
                        if sigma_best is not None:
                            print("Can connect to goal")

                            return sigma_best, E_a, E_b
                        else:
                            print("Could not connect to goal")

                        return sigma_best, E_a, E_b

            # swap trees
            V_a, V_b = V_b, V_a
            V_a_count, V_b_count = V_b_count, V_a_count
            E_a, E_b = E_b, E_a
            x_init, x_goal = x_goal, x_init
