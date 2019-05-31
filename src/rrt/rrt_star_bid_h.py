# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import random

from src.rrt.rrt_star_bid import RRTStarBidirectional
from src.utilities.geometry import dist_between_points, pairwise


class RRTStarBidirectionalHeuristic(RRTStarBidirectional):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01,
                 rewire_count: int = None, conditional_rewire: bool = False):
        """
        Bidirectional RRT* Search
        :param X: Search Space
        :param Q: list of lengths of edges added to tree
        :param x_init: tuple, initial location
        :param x_goal: tuple, goal location
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        :param rewire_count: number of nearby vertices to rewire
        :param conditional_rewire: if True, set rewire count to 1 until solution found,
        then set to specified rewire count (ensure runtime complexity guarantees)
        """
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc,
                         1 if conditional_rewire else rewire_count)
        self.original_rewire_count = rewire_count

    def rrt_star_bid_h(self):
        """
        Bidirectional RRT* using added heuristics
        :return: set of Vertices; Edges in form: vertex: [neighbor_1, neighbor_2, ...]
        """
        # tree a
        self.add_vertex(0, self.x_init)
        self.add_edge(0, self.x_init, None)

        # tree b
        self.add_tree()
        self.add_vertex(1, self.x_goal)
        self.add_edge(1, self.x_goal, None)

        while True:
            for q in self.Q:  # iterate over different edge lengths
                for i in range(q[1]):  # iterate over number of edges of given length to add
                    x_new, x_nearest = self.new_and_near(0, q)
                    if x_new is None:
                        continue

                    # get nearby vertices and cost-to-come
                    L_near = self.get_nearby_vertices(0, self.x_init, x_new)

                    # check nearby vertices for total cost and connect shortest valid edge
                    self.connect_shortest_valid(0, x_new, L_near)

                    if x_new in self.trees[0].E:
                        # rewire tree
                        self.rewire(0, x_new, L_near)

                        # nearby vertices from opposite tree and cost-to-come
                        L_near = self.get_nearby_vertices(1, self.x_goal, x_new)

                        self.connect_trees(0, 1, x_new, L_near)
                        self.rewire_count = self.original_rewire_count

                    self.lazy_shortening()

                    if self.prc and random.random() < self.prc:  # probabilistically check if solution found
                        print("Checking if can connect to goal at", str(self.samples_taken), "samples")
                        if self.sigma_best is not None:
                            print("Can connect to goal")
                            self.unswap()

                            return self.sigma_best

                    if self.samples_taken >= self.max_samples:
                        self.unswap()

                        if self.sigma_best is not None:
                            print("Can connect to goal")

                            return self.sigma_best
                        else:
                            print("Could not connect to goal")

                        return self.sigma_best

            self.swap_trees()

    def lazy_shortening(self):
        """
        Lazily attempt to shorten current best path
        """
        if self.sigma_best is not None and len(self.sigma_best) > 2:
            a, b = 0, 0
            while not abs(a - b) > 1:
                a, b = random.sample(range(0, len(self.sigma_best)), 2)

            a, b = min(a, b), max(a, b)
            v_a, v_b = tuple(self.sigma_best[a]), tuple(self.sigma_best[b])

            if self.X.collision_free(v_a, v_b, self.r):
                # create new edge connecting vertices
                if v_a in self.trees[0].E and v_b in self.reconstruct_path(0, self.x_init, v_a):
                    self.trees[0].E[v_a] = v_b
                elif v_a in self.trees[1].E and v_b in self.reconstruct_path(1, self.x_goal, v_a):
                    self.trees[1].E[v_a] = v_b
                elif v_b in self.trees[0].E and v_a in self.reconstruct_path(0, self.x_init, v_b):
                    self.trees[0].E[v_b] = v_a
                elif v_b in self.trees[1].E and v_a in self.reconstruct_path(1, self.x_goal, v_b):
                    self.trees[1].E[v_b] = v_a
                elif v_a in self.trees[0].E:
                    self.trees[0].E[v_b] = v_a
                else:
                    self.trees[1].E[v_b] = v_a

                # update best path
                # remove cost of removed edges
                self.c_best -= sum(dist_between_points(i, j) for i, j in pairwise(self.sigma_best[a:b + 1]))
                # add cost of new edge
                self.c_best += dist_between_points(self.sigma_best[a], self.sigma_best[b])
                self.sigma_best = self.sigma_best[:a + 1] + self.sigma_best[b:]
