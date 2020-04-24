# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import random

from src.rrt.heuristics import path_cost
from src.rrt.rrt_star import RRTStar


class RRTStarBidirectional(RRTStar):
    def __init__(self, X, Q, x_init, x_goal, max_samples, r, prc=0.01, rewire_count=None):
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
        """
        super().__init__(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
        self.sigma_best = None  # best solution thus far
        self.swapped = False

    def connect_trees(self, a, b, x_new, L_near):
        """
        Check nearby vertices for total cost and connect shortest valid edge if possible
        This results in both trees being connected
        :param a: first tree to connect
        :param b: second tree to connect
        :param x_new: new vertex to add
        :param L_near: nearby vertices
        """
        for c_near, x_near in L_near:
            c_tent = c_near + path_cost(self.trees[a].E, self.x_init, x_new)
            if c_tent < self.c_best and self.X.collision_free(x_near, x_new, self.r):
                self.trees[b].V_count += 1
                self.trees[b].E[x_new] = x_near
                self.c_best = c_tent
                sigma_a = self.reconstruct_path(a, self.x_init, x_new)
                sigma_b = self.reconstruct_path(b, self.x_goal, x_new)
                del sigma_b[-1]
                sigma_b.reverse()
                self.sigma_best = sigma_a + sigma_b

                break

    def swap_trees(self):
        """
        Swap trees and start/goal
        """
        # swap trees
        self.trees[0], self.trees[1] = self.trees[1], self.trees[0]
        # swap start/goal
        self.x_init, self.x_goal = self.x_goal, self.x_init
        self.swapped = not self.swapped

    def unswap(self):
        """
        Check if trees have been swapped and unswap
        Reverse path if needed to correspond with swapped trees
        """
        if self.swapped:
            self.swap_trees()

        if self.sigma_best is not None and self.sigma_best[0] is not self.x_init:
            self.sigma_best.reverse()

    def rrt_star_bidirectional(self):
        """
        Bidirectional RRT*
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
