import numpy as np

class DWA:
    def __init__(self, config):
        self.config = config

    def plan(self, current_position, goal_position, X, obstacles):
        # Generate the dynamic window based on current speed and robot limitations
        dw = self.calc_dynamic_window(current_position)
        
        # Evaluate trajectories in the dynamic window
        best_u = None
        min_cost = float('inf')
        best_trajectory = []

        for v, w in dw:
            trajectory = self.generate_trajectory(current_position, v, w)
            cost = self.calc_cost(trajectory, goal_position, obstacles)

            if cost < min_cost:
                min_cost = cost
                best_u = (v, w)
                best_trajectory = trajectory

        return best_trajectory

    def calc_dynamic_window(self, current_position):
        # Dynamic window calculation
        v_min = max(self.config['min_speed'], current_position[3] - self.config['max_accel'] * self.config['dt'])
        v_max = min(self.config['max_speed'], current_position[3] + self.config['max_accel'] * self.config['dt'])
        w_min = max(-self.config['max_yaw_rate'], current_position[4] - self.config['max_accel'] * self.config['dt'])
        w_max = min(self.config['max_yaw_rate'], current_position[4] + self.config['max_accel'] * self.config['dt'])
        
        return [(v, w) for v in np.arange(v_min, v_max, self.config['v_reso']) for w in np.arange(w_min, w_max, self.config['yaw_rate_reso'])]

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

