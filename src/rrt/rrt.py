from rtree import index

from src.configuration_space.configuration_space import ConfigurationSpace


class RRT(object):
    def __init__(self, X: ConfigurationSpace, Q: list, max_samples: int, r: float, prc: float = 0.01):
        """
        Template RRT planner
        :param X: Configuration Space
        :param Q: list of lengths of edges added to tree
        :param max_samples: max number of samples to take
        :param r: resolution of points to sample along edge when checking for collisions
        :param prc: probability of checking whether there is a solution
        """
        self.X = X
        self.max_samples = max_samples
        self.Q = Q
        self.r = r
        self.prc = prc

        p = index.Property()
        p.dimension = self.X.dimensions
        self.V = index.Index(interleaved=True, properties=p)
        self.V_count = 1
        self.samples_taken = 0
        self.E = {}
