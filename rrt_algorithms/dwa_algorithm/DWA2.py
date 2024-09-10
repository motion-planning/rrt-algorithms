# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
#the RRT and DWA algorithms function with the optimized collision checking using Bounding Volume Hierarchies (BVH).
#BVH Implementation: We use an R-tree (a type of BVH) to store axis-aligned bounding boxes (AABB) for each obstacle. This allows us to quickly identify which obstacles might intersect with a given point or path segment.
#Efficient Collision Checking: Instead of checking every obstacle for collision, we first use the BVH to narrow down the list to only those obstacles whose bounding boxes intersect with the point or path segment in question.
#Integration with DWA: The code remains the same in terms of how paths are generated and optimized, but now the collision checking is more efficient, leading to faster overall performance.
#All obstacles (both random and predefined) need to be properly registered in the SearchSpace so that the collision checking functions (obstacle_free and collision_free) are aware of all obstacles.
# the script avoids both static and dynamic obstacles, it starts from orange dot and its final goal is green.
import sys
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Update the DWA plan method to use the fuzzy controller
class FuzzyController:
    def __init__(self):
        # Define fuzzy variables
        self.distance_to_goal = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_to_goal')
        self.distance_to_obstacle = ctrl.Antecedent(np.arange(0, 101, 1), 'distance_to_obstacle')
        
        self.to_goal_gain = ctrl.Consequent(np.arange(0, 11, 1), 'to_goal_gain')
        self.speed_gain = ctrl.Consequent(np.arange(0, 11, 1), 'speed_gain')
        self.obstacle_gain = ctrl.Consequent(np.arange(0, 11, 1), 'obstacle_gain')
        
        # Auto-membership function with 3 levels: 'poor', 'average', 'good'
        self.distance_to_goal.automf(3)
        self.distance_to_obstacle.automf(3)
        
        self.to_goal_gain.automf(3)
        self.speed_gain.automf(3)
        self.obstacle_gain.automf(3)
        
        # Define fuzzy rules to cover all cases
        rule1 = ctrl.Rule(self.distance_to_goal['poor'] & self.distance_to_obstacle['good'], 
                          (self.to_goal_gain['good'], self.speed_gain['good'], self.obstacle_gain['poor']))
        rule2 = ctrl.Rule(self.distance_to_goal['average'] & self.distance_to_obstacle['average'], 
                          (self.to_goal_gain['average'], self.speed_gain['average'], self.obstacle_gain['average']))
        rule3 = ctrl.Rule(self.distance_to_goal['good'] & self.distance_to_obstacle['poor'], 
                          (self.to_goal_gain['poor'], self.speed_gain['poor'], self.obstacle_gain['good']))

        # Create control system
        self.control_system = ctrl.ControlSystem([rule1, rule2, rule3])
        self.simulation = ctrl.ControlSystemSimulation(self.control_system)
    
    def compute_gains(self, dist_to_goal, dist_to_obstacle):
        # Set inputs
        self.simulation.input['distance_to_goal'] = dist_to_goal
        self.simulation.input['distance_to_obstacle'] = dist_to_obstacle
        
        # Perform fuzzy computation
        self.simulation.compute()
        
        # Ensure the fuzzy system has outputted the necessary gains
        return (self.simulation.output.get('to_goal_gain', 0), 
                self.simulation.output.get('speed_gain', 0), 
                self.simulation.output.get('obstacle_gain', 0))
                
class DWA:
    def __init__(self, config):
        self.config = config
        self.fuzzy_controller = FuzzyController()

    def plan(self, start_point, end_point, X, obstacles):
        # Compute distance to goal and nearest obstacle
        dist_to_goal = np.linalg.norm(np.array(start_point[:2]) - np.array(end_point[:2]))
        dist_to_obstacle = min([np.linalg.norm(np.array(start_point[:2]) - np.array(obs[:2])) - obs[4] for obs in obstacles])

        # Get fuzzy-adjusted weights
        to_goal_gain, speed_gain, obstacle_gain = self.fuzzy_controller.compute_gains(dist_to_goal, dist_to_obstacle)
        # Generate dynamic window and evaluate possible trajectories
        dw = self.calc_dynamic_window(start_point)
        best_trajectory = None
        min_cost = float('inf')
    
        for v in np.arange(dw[0], dw[1], self.config['v_reso']):
            for y in np.arange(dw[2], dw[3], self.config['yaw_rate_reso']):
                trajectory = self.generate_trajectory(start_point, v, y)
                if X.collision_free(trajectory[0], trajectory[-1], self.config['dt']):
                    cost = (to_goal_gain * self.calc_to_goal_cost(trajectory[-1], end_point) + 
                            speed_gain * (self.config['max_speed'] - trajectory[-1][3]) + 
                            obstacle_gain * self.calc_obstacle_cost(trajectory, obstacles))
                    if cost < min_cost:
                        min_cost = cost
                        best_trajectory = trajectory

        return best_trajectory

    def calc_dynamic_window(self, current_position):
        # Dynamic window calculation, returning individual scalar values instead of a list of tuples
        v_min = max(self.config['min_speed'], current_position[3] - self.config['max_accel'] * self.config['dt'])
        v_max = min(self.config['max_speed'], current_position[3] + self.config['max_accel'] * self.config['dt'])
        w_min = max(-self.config['max_yaw_rate'], current_position[4] - self.config['max_accel'] * self.config['dt'])
        w_max = min(self.config['max_yaw_rate'], current_position[4] + self.config['max_accel'] * self.config['dt'])
        
        return [v_min, v_max, w_min, w_max]

    def generate_trajectory(self, current_position, v, w):
        # Generate trajectory for a given velocity and yaw rate
        trajectory = [current_position]
        time = 0

        while time <= self.config['predict_time']:
            next_position = self.motion_model(trajectory[-1], v, w)
            trajectory.append(next_position)
            time += self.config['dt']

        return trajectory

    def motion_model(self, state, v, w):
        # Update the state [x, y, theta, v, w] given current velocities
        x, y, theta, _, _ = state
        x += v * np.cos(theta) * self.config['dt']
        y += v * np.sin(theta) * self.config['dt']
        theta += w * self.config['dt']
        return [x, y, theta, v, w]

    def calc_cost(self, trajectory, goal_position, obstacles):
        # Cost function combining distance to goal, speed, and distance to obstacles
        to_goal_cost = self.config['to_goal_cost_gain'] * self.calc_to_goal_cost(trajectory[-1], goal_position)
        speed_cost = self.config['speed_cost_gain'] * (self.config['max_speed'] - trajectory[-1][3])
        obstacle_cost = self.config['obstacle_cost_gain'] * self.calc_obstacle_cost(trajectory, obstacles)

        return to_goal_cost + speed_cost + obstacle_cost

    def calc_to_goal_cost(self, state, goal_position):
        # Euclidean distance to the goal
        return np.linalg.norm([state[0] - goal_position[0], state[1] - goal_position[1]])

    def calc_obstacle_cost(self, trajectory, obstacles):
        # Calculate the distance to the closest obstacle
        min_dist = float('inf')
        for point in trajectory:
            for obs in obstacles:
                dist = np.linalg.norm([point[0] - obs[0], point[1] - obs[1]])
                if dist < min_dist:
                    min_dist = dist
        return min_dist

# Sample usage of DWA with RRT
"""
if __name__ == "__main__":
    dwa_params = {
        'max_speed': 1.0,
        'min_speed': 0.0,
        'max_yaw_rate': 40.0 * np.pi / 180.0,
        'max_accel': 0.2,
        'v_reso': 0.01,
        'yaw_rate_reso': 0.1 * np.pi / 180.0,
        'dt': 0.1,
        'predict_time': 3.0,
        'to_goal_cost_gain': 1.0,
        'speed_cost_gain': 1.0,
        'obstacle_cost_gain': 1.0,
    }
    
    dwa = DWA(dwa_params)
    current_position = [0, 0, 0, 0, 0]  # Example starting position [x, y, theta, v, w]
    goal_position = [10, 10]  # Example goal position [x, y]
    X = None  # The search space; not used in this basic example
    obstacles = [(5, 5), (7, 8)]  # Example obstacles; replace with actual obstacle positions

    trajectory = dwa.plan(current_position, goal_position, X, obstacles)
    print("Best trajectory:", trajectory)
"""
