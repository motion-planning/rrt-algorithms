# Texpanding the radius of the obstacles by the radius of the drone (10 cm) to ensure that the drone does not collide with any obstacles. This adjustment effectively treats the drone as a point but expands the obstacles to account for the drone's size.
import numpy as np
import time
from rtree import index
import sys
import uuid
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
sys.path.append('/home/dell/rrt-algorithms')

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space2 import SearchSpace
from rrt_algorithms.utilities.plotting2 import Plot
from rrt_algorithms.dwa_algorithm.DWA3 import DWA, FuzzyController
from rrt_algorithms.dqn_algorithm.DQN import DQNAgent
from rrt_algorithms.dqn_algorithm.DQN import DroneEnv
from rrt_algorithms.utilities.obstacle_generation2 import generate_random_cylindrical_obstacles 
from tensorflow.keras.models import load_model

# Define the search space dimensions
X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])
X = SearchSpace(X_dimensions)
# Initial and goal positions
x_init = (50, 50, -2)
x_intermediate = (50, 50, 100)
x_second_goal = (20, 60, 100)
x_final_goal = (0, 0, 0)
#x_final_goal = (100, 100, 40)
# Create the search space


# Generate random obstacles
n = 7  # Number of random obstacles
Obstacles = generate_random_cylindrical_obstacles(X, x_init, x_final_goal, n)

# Define static obstacles
obstacles = [
    (10, 82, 0, 85, 15),  
    (80, 20, 0, 60, 19),   
    (10, 30, 0, 90, 15),   
    (80, 80, 0, 70, 22), 
    (50, 50, 0, 50, 22),
]

# Combine static and randomly generated obstacles into one list
all_obstacles = obstacles + Obstacles
# Initialize the SearchSpace with all obstacles
X = SearchSpace(X_dimensions, all_obstacles)

# RRT parameters
q = 700
r = 1
max_samples = 1024
prc = 0.1

# DWA parameters
dwa_params = {
    'max_speed': 1.0,
    'min_speed': 0.0,
    'max_yaw_rate': 40.0 * np.pi / 180.0,
    'max_accel': 0.3,  
    'v_reso': 0.05,  
    'yaw_rate_reso': 0.2 * np.pi / 180.0,
    'dt': 0.1,
    'predict_time': 2.0,
    'to_goal_cost_gain': 1.5,
    'speed_cost_gain': 0.5,
    'obstacle_cost_gain': 1.0,
}
def compute_energy_usage(path, velocity):
    energy = 0.0
    for i in range(1, len(path)):
        distance = np.linalg.norm(np.array(path[i]) - np.array(path[i-1]))
        energy += distance * velocity
    return energy
    
def min_obstacle_clearance(path, obstacles):
        min_clearance = float('inf')
        for point in path:
            for obs in obstacles:
                clearance = np.linalg.norm(np.array(point[:2]) - np.array(obs[:2])) - obs[4]
                if clearance < min_clearance:
                    min_clearance = clearance
        return min_clearance   
         
def path_length(path):
    return sum(np.linalg.norm(np.array(path[i+1]) - np.array(path[i])) for i in range(len(path) - 1))

def path_smoothness(path):
    total_curvature = 0.0
    for i in range(1, len(path) - 1):
        vec1 = np.array(path[i]) - np.array(path[i-1])
        vec2 = np.array(path[i+1]) - np.array(path[i])
        angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
        total_curvature += angle
    return total_curvature
    
# RRT pathfinding to first intermediate goal
print("RRT to first intermediate goal...")
start_time = time.time()
rrt = RRT(X, q, x_init, x_intermediate, max_samples, r, prc)
path1 = rrt.rrt_search()
rrt_time = time.time() - start_time

if path1 is not None:
    print("RRT to second intermediate goal...")
    rrt = RRT(X, q, x_intermediate, x_second_goal, max_samples, r, prc)
    path2 = rrt.rrt_search()

    if path2 is not None:
        print("RRT to final goal...")
        rrt = RRT(X, q, x_second_goal, x_final_goal, max_samples, r, prc)
        path3 = rrt.rrt_search()

        # Combine all paths
        if path3 is not None:
            path = path1 + path2[1:] + path3[1:]
        else:
            path = path1 + path2[1:]
    else:
        path = path1
else:
    path = None

# Apply DWA for local optimization along the RRT path
if path is not None:
    dwa = DWA(dwa_params)
    optimized_path = []

    start_time = time.time()
    for i in range(len(path) - 1):
        start_point = path[i] + (0.0, 0.0)  # Initialize v and w to 0, using tuple
        end_point = path[i + 1]

        local_path = dwa.plan(start_point, end_point, X, all_obstacles)
        optimized_path.extend(local_path)
    dwa_time = time.time() - start_time

# Metrics calculation and plotting
rrt_path_length = path_length(path)
dwa_path_length = path_length(optimized_path)

rrt_smoothness = path_smoothness(path)
dwa_smoothness = path_smoothness(optimized_path)

rrt_clearance = min_obstacle_clearance(path, all_obstacles)
dwa_clearance = min_obstacle_clearance(optimized_path, all_obstacles)

average_velocity = dwa_params['max_speed']
rrt_energy = compute_energy_usage(path, average_velocity)
dwa_energy = compute_energy_usage(optimized_path, average_velocity)
# Initialize the DroneEnv environment

# Training the DQN
env = DroneEnv(X, x_init, x_final_goal, all_obstacles, dwa_params)
env.train(episodes=500)
# After training, use the learned policy in your DWA
state = env.reset()
done = False
optimized_path = []

while not done:
    action = env.agent.act(state)
    next_state, _, done = env.step(action)
    optimized_path.append(next_state)
    state = next_state

# Initialize the DQN Agent
agent = DQNAgent(env.state_size, env.action_size)

# Train the DQN agent
#agent.train(env, episodes=10)

# Save the trained model
agent.model.save("dqn_model.keras")
# Print Metrics
print(f"RRT Path Length: {rrt_path_length}")
print(f"DWA Optimized Path Length: {dwa_path_length}")
print(f"RRT Path Smoothness: {rrt_smoothness}")
print(f"DWA Optimized Path Smoothness: {dwa_smoothness}")
print(f"RRT Path Minimum Clearance: {rrt_clearance}")
print(f"DWA Optimized Path Minimum Clearance: {dwa_clearance}")
print(f"RRT Path Energy Usage: {rrt_energy}")
print(f"DWA Optimized Path Energy Usage: {dwa_energy}")


# Plotting
plot = Plot("rrt_dwa_3d")
plot.plot_tree(X, rrt.trees)
plot.plot_path(X, path, color='red')  # Plot the original RRT path in red
for point in optimized_path:
    print("Point in optimized_path:", point)

# Filter out incorrectly structured points (e.g., points that aren't tuples or lists with at least two elements)
filtered_optimized_path = [point for point in optimized_path if isinstance(point, (list, tuple)) and len(point) >= 2]

# Now plot using the filtered path
plot.plot_path(X, filtered_optimized_path, color='blue')  # Plot the optimized path in blue

plot.plot_obstacles(X, all_obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_intermediate, color="pink")
plot.plot_goal(X, x_second_goal, color="blue")
plot.plot_goal(X, x_final_goal, color="green")
plot.draw(auto_open=True)

