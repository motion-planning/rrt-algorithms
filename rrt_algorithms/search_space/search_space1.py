# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#the RRT and DWA algorithms function with the optimized collision checking using Bounding Volume Hierarchies (BVH).
#BVH Implementation: We use an R-tree (a type of BVH) to store axis-aligned bounding boxes (AABB) for each obstacle. This allows us to quickly identify which obstacles might intersect with a given point or path segment.
#Efficient Collision Checking: Instead of checking every obstacle for collision, we first use the BVH to narrow down the list to only those obstacles whose bounding boxes intersect with the point or path segment in question.
#Integration with DWA: The code remains the same in terms of how paths are generated and optimized, but now the collision checking is more efficient, leading to faster overall performance.
#All obstacles (both random and predefined) need to be properly registered in the SearchSpace so that the collision checking functions (obstacle_free and collision_free) are aware of all obstacles.
# the script avoids both static and dynamic obstacles, it starts from orange dot and its final goal is green.
import numpy as np
from rtree import index

from rrt_algorithms.utilities.geometry import es_points_along_line
from rrt_algorithms.utilities.obstacle_generation1 import obstacle_generator

from rtree import index

class SearchSpace:
    def __init__(self, dimension_lengths, obstacles=None):
        self.dimension_lengths = dimension_lengths
        self.obstacles = obstacles if obstacles is not None else []
        self.dimensions = len(dimension_lengths)  # Add this line to define the number of dimensions
        self.build_bvh()
        self.build_obs_index() 
        
    def build_bvh(self):
        # Create a BVH tree for fast collision detection
        p = index.Property()
        p.dimension = 3  # Set dimension to 3 for 3D bounding boxes
        self.bvh = index.Index(properties=p)
        for i, obstacle in enumerate(self.obstacles):
            aabb = self.compute_aabb(obstacle)
            self.bvh.insert(i, aabb)

    def build_obs_index(self):
        # Create an R-tree index for obstacles
        p = index.Property()
        p.dimension = 3  # Set dimension to 3 for 3D obstacles
        self.obs = index.Index(properties=p)
        for i, obstacle in enumerate(self.obstacles):
            aabb = self.compute_aabb(obstacle)
            self.obs.insert(i, aabb)
            
    def compute_aabb(self, obstacle):
        # Compute the Axis-Aligned Bounding Box (AABB) for the obstacle
        x_center, y_center, z_min, z_max, radius = obstacle
        x_min = x_center - radius
        x_max = x_center + radius
        y_min = y_center - radius
        y_max = y_center + radius
        z_min = z_min  # Lower z-bound
        z_max = z_max  # Upper z-bound
        return (x_min, y_min, z_min, x_max, y_max, z_max)

    def sample(self):
        """
        Return a random location within the search space dimensions.
        :return: A tuple representing a random location within the search space.
        """
        return tuple(np.random.uniform(low, high) for low, high in self.dimension_lengths)
    def collision_free(self, start, end, r):
        direction = np.array(end) - np.array(start)
        length = np.linalg.norm(direction)
        direction = direction / length
        steps = int(length / r)
        for i in range(steps + 1):
            point = np.array(start) + i * r * direction
            if not self.obstacle_free(point):
                return False
        return True



    def is_point_in_obstacle(self, point, obstacle):
        # Detailed check if the point is inside the actual obstacle
        x_center, y_center, z_min, z_max, radius = obstacle
        dist = np.linalg.norm(np.array(point[:2]) - np.array([x_center, y_center]))
        return dist <= radius and z_min <= point[2] <= z_max



    def es_points_along_line(self, x_a, x_b, r):
        """
        Generate points along a line segment between x_a and x_b.
        :param x_a: Starting point of the line segment.
        :param x_b: Ending point of the line segment.
        :param r: Resolution for sampling points.
        :return: List of points along the line segment.
        """
        distance = np.linalg.norm(np.array(x_b) - np.array(x_a))
        num_points = int(distance / r) + 1
        return [tuple(np.array(x_a) + t * (np.array(x_b) - np.array(x_a))) for t in np.linspace(0, 1, num_points)]




    def obstacle_free(self, point):
        """
        Check if a point is free from obstacles.
        :param point: A tuple representing a point in the search space.
        :return: True if the point is free from obstacles, False otherwise.
        """
        nearby_obstacles = list(self.bvh.intersection((point[0], point[1], point[2], point[0], point[1], point[2])))
        for obs_idx in nearby_obstacles:
            if self.is_point_in_obstacle(point, self.obstacles[obs_idx]):
                return False
        return True

    def sample_free(self):
        """
        Sample a random location within the search space that is free from obstacles.
        :return: A tuple representing a random location within the search space that is obstacle-free.
        """
        while True:
            point = self.sample()
            if self.obstacle_free(point):
                return point
