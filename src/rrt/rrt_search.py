# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import random
import uuid

from src.configuration_space.configuration_space import ConfigurationSpace
from src.rrt.primitive_procedures import can_connect_to_goal, reconstruct_path
from src.rrt.primitive_procedures import connect_to_goal
from src.rrt.primitive_procedures import steer
from src.rrt.rrt import RRT


class RRTSearch(RRT):
    def __init__(self, X: ConfigurationSpace, Q: list, max_samples: int, r: float, prc: float = None):
        """
        RRT Search
        :param X: Configuration Space
        :param Q: list of lengths of edges added to tree
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        """
        super().__init__(X, Q, max_samples, r, prc)

    def rrt_search(self, x_init: tuple, x_goal: tuple) -> (list, dict):
        """
        Create and return a Rapidly-exploring Random Tree, keeps expanding until can connect to goal
        https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
        :param x_init: initial location
        :param x_goal: goal location
        :return: list representation of path, dict representing edges of tree in form E[child] = parent
        """
        self.V.insert(1, x_init + x_init, x_init)
        self.E = {x_init: None}

        while True:
            for q in self.Q:  # iterate over different edge lengths
                for i in range(q[1]):  # iterate over number of edges of given length to add
                    x_rand = self.X.sample_free()
                    x_nearest = list(self.V.nearest(x_rand, num_results=1, objects="raw"))[0]
                    x_new = steer(self.X, x_nearest, x_rand, q[0])
                    # check if new point is in X_free and not already in V
                    if not self.X.obstacle_free(x_new) or not self.V.count(x_new) == 0:
                        continue

                    if self.V.count(x_new) == 0 and self.X.collision_free(x_nearest, x_new, self.r):
                        self.V.insert(uuid.uuid4(), x_new + x_new, x_new)
                        self.V_count += 1
                        self.E[x_new] = x_nearest

                        self.samples_taken += 1

                    if self.prc and random.random() < self.prc:  # randomly check if solution found
                        print("Checking if can connect to goal at", str(self.samples_taken), "samples")
                        if can_connect_to_goal(self.X, self.V, x_goal, self.Q, self.r):
                            print("Can connect to goal")
                            self.E = connect_to_goal(self.V, self.E, x_goal)
                            path = reconstruct_path(self.E, x_init, x_goal)

                            return path, self.E

                    if self.samples_taken >= self.max_samples:
                        if can_connect_to_goal(self.X, self.V, x_goal, self.Q, self.r):
                            print("Can connect to goal")
                            self.E = connect_to_goal(self.V, self.E, x_goal)
                            path = reconstruct_path(self.E, x_init, x_goal)

                            return path, self.E
                        else:
                            print("Could not connect to goal")

                        return None, self.E
