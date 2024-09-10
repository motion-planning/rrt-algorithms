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
import random
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
    

# Adaptive Large Neighborhood Search (ALNS) Functions
def alns_optimize_path(path, X, all_obstacles, max_iterations=100):
    """
    Optimize the path using ALNS with adaptive neighborhoods.
    :param path: Initial path to optimize.
    :param X: Search space object.
    :param all_obstacles: List of all obstacles in the environment.
    :param max_iterations: Maximum number of iterations for ALNS.
    :return: Optimized path.
    """
    neighborhoods = [segment_change, detour_addition, direct_connection]
    neighborhood_scores = {segment_change: 1.0, detour_addition: 1.0, direct_connection: 1.0}
    current_path = path.copy()

    for _ in range(max_iterations):
        # Select neighborhood adaptively
        neighborhood = random.choices(neighborhoods, weights=neighborhood_scores.values())[0]
        new_path = neighborhood(current_path, X, all_obstacles)

        # Evaluate the new path
        new_path_length = path_length(new_path)
        current_path_length = path_length(current_path)

        if new_path_length < current_path_length:
            current_path = new_path
            neighborhood_scores[neighborhood] *= 1.1  # Reward successful neighborhood
        else:
            neighborhood_scores[neighborhood] *= 0.9  # Penalize unsuccessful neighborhood

    return current_path

def segment_change(path, X, all_obstacles, r=1):
    if len(path) < 3:
        return path

    i = random.randint(0, len(path) - 3)
    j = random.randint(i + 2, len(path) - 1)

    x_a = path[i]
    x_b = path[j]

    spatial_x_a = np.array(x_a[:3])
    spatial_x_b = np.array(x_b[:3])

    new_point_spatial = spatial_x_a + (spatial_x_b - spatial_x_a) / 2
    new_point = list(new_point_spatial) + list(x_a[3:])

    if X.collision_free(spatial_x_a, new_point_spatial, r) and X.collision_free(new_point_spatial, spatial_x_b, r):
        new_path = path[:i + 1] + [new_point] + path[j:]
        return new_path

    return path

def detour_addition(path, X, all_obstacles, r=1):
    if len(path) < 2:
        return path

    i = random.randint(0, len(path) - 2)
    x_a = path[i]
    x_b = path[i + 1]

    spatial_x_a = np.array(x_a[:3])
    spatial_x_b = np.array(x_b[:3])

    detour_point_3d = spatial_x_a + (np.random.rand(3) - 0.5) * 2 * r
    detour_point = list(detour_point_3d) + list(x_a[3:])

    if X.collision_free(spatial_x_a, detour_point_3d, r) and X.collision_free(detour_point_3d, spatial_x_b, r):
        new_path = path[:i + 1] + [detour_point] + path[i + 1:]
        return new_path

    return path


def direct_connection(path, X, all_obstacles, r=1):
    """
    Directly connect two non-adjacent points to shorten the path.
    """
    if len(path) < 3:
        return path  # Can't directly connect if there aren't enough points

    new_path = path.copy()
    i = random.randint(0, len(path) - 3)  # Select a random starting point
    j = random.randint(i + 2, len(path) - 1)  # Select a random ending point

    x_a = path[i][:3]  # Extract only the spatial coordinates (x, y, z)
    x_b = path[j][:3]  # Extract only the spatial coordinates (x, y, z)

    if X.collision_free(x_a, x_b, r):
        new_path = new_path[:i + 1] + new_path[j:]  # Remove the points between i and j

    return new_path



# RRT pathfinding to the first intermediate goal
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

    # Apply ALNS for global path optimization
    alns_optimized_path = alns_optimize_path(optimized_path, X, all_obstacles, max_iterations=100)

    # Ensure all paths are correctly formatted and flatten nested structures
    def validate_and_correct_path(path):
        corrected_path = []
        for idx, point in enumerate(path):
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                if isinstance(point[0], (list, tuple)):
                    point = [item for sublist in point for item in sublist]
                corrected_path.append(point[:3])  # Ensure only the first three elements are used
            else:
                raise ValueError(f"Point {idx} in path is incorrectly formatted: {point}")
        return corrected_path

    # Validate and correct paths
    optimized_path = validate_and_correct_path(optimized_path)
    alns_optimized_path = validate_and_correct_path(alns_optimized_path)

    # Metrics calculation
    rrt_path_length = path_length(path)
    dwa_path_length = path_length(optimized_path)
    alns_path_length = path_length(alns_optimized_path)

    rrt_smoothness = path_smoothness(path)
    dwa_smoothness = path_smoothness(optimized_path)
    alns_smoothness = path_smoothness(alns_optimized_path)

    rrt_clearance = min_obstacle_clearance(path, all_obstacles)
    dwa_clearance = min_obstacle_clearance(optimized_path, all_obstacles)
    alns_clearance = min_obstacle_clearance(alns_optimized_path, all_obstacles)

    average_velocity = dwa_params['max_speed']
    rrt_energy = compute_energy_usage(path, average_velocity)
    dwa_energy = compute_energy_usage(optimized_path, average_velocity)
    alns_energy = compute_energy_usage(alns_optimized_path, average_velocity)
    # Training the DQN
    env = DroneEnv(X, x_init, x_final_goal, all_obstacles, dwa_params)
    env.train(episodes=10)
    # After training, use the learned policy in your DWA
  
    # Initialize the DroneEnv environment for path optimization
    state = env.reset()
    done = False
    optimized_path = []
agent = DQNAgent(env.state_size, env.action_size)






while not done:
    action = agent.act(state)
    next_state, _, done = env.step(action)
    optimized_path.append(next_state)
    state = next_state
    # Save the trained DQN model
    #agent = DQNAgent(env.state_size, env.action_size)
    # Save the trained model
    agent.model.save("dqn_model.keras")
    print("DQN model saved as dqn_model.keras")
    
    

    
    
    # Print Metrics
    print(f"RRT Path Length: {rrt_path_length}")
    print(f"DWA Optimized Path Length: {dwa_path_length}")
    print(f"ALNS Optimized Path Length: {alns_path_length}")
    print(f"RRT Path Smoothness: {rrt_smoothness}")
    print(f"DWA Optimized Path Smoothness: {dwa_smoothness}")
    print(f"ALNS Optimized Path Smoothness: {alns_smoothness}")
    print(f"RRT Path Minimum Clearance: {rrt_clearance}")
    print(f"DWA Optimized Path Minimum Clearance: {dwa_clearance}")
    print(f"ALNS Optimized Path Minimum Clearance: {alns_clearance}")
    print(f"RRT Path Energy Usage: {rrt_energy}")
    print(f"DWA Optimized Path Energy Usage: {dwa_energy}")
    print(f"ALNS Optimized Path Energy Usage: {alns_energy}")
    

def inspect_and_print_path(path):
    """
    Inspects and prints the structure of the path for debugging.
    """
    for idx, point in enumerate(path):
        print(f"Point {idx} in path: {point} (Type: {type(point)})")
        if isinstance(point, (list, tuple)):
            for sub_idx, sub_point in enumerate(point):
                print(f"  Sub-point {sub_idx}: {sub_point} (Type: {type(sub_point)})")
def inspect_path(path, path_name):
    if path is None or len(path) == 0:
        print(f"{path_name} is empty or None.")
    else:
        print(f"{path_name} contains {len(path)} points. Sample points: {path[:5]}")

# Inspect paths
inspect_path(path1, "RRT Path 1")
inspect_path(path2, "RRT Path 2")
inspect_path(path3, "RRT Path 3")
inspect_path(optimized_path, "DWA Optimized Path")
inspect_path(alns_optimized_path, "ALNS Optimized Path")

def flatten_path_points(path):
    """
    Flattens any nested numpy arrays in the path and ensures that each point is a list or tuple with at least three elements.
    Returns a cleaned and flattened path.
    """
    cleaned_path = []
    for idx, point in enumerate(path):
        if isinstance(point, np.ndarray):
            # Flatten the array and convert to a list
            flat_point = point.flatten().tolist()
            if len(flat_point) >= 3:
                cleaned_path.append(flat_point[:3])  # Take only the first three elements (x, y, z)
            else:
                raise ValueError(f"Point {idx} in optimized_path is incorrectly formatted: {flat_point}")
        else:
            raise ValueError(f"Point {idx} in optimized_path is not a list or tuple: {point}")
    return cleaned_path

# Apply the flattening function to optimized_path before plotting
try:
    optimized_path = flatten_path_points(optimized_path)
except ValueError as e:
    print(f"Error encountered during flattening: {e}")


# Debug the flattened path points
for idx, point in enumerate(optimized_path):
   print(f"Point {idx} in optimized_path after flattening: {point}")

# Plotting
plot = Plot("rrt_dwa_alns_3d")

# Your existing plotting code follows
plot.plot_tree(X, rrt.trees)
plot.plot_path(X, path, color='red')  # Plot the original RRT path in red
plot.plot_path(X, optimized_path, color='blue')  # Plot the DWA optimized path in blue
plot.plot_path(X, alns_optimized_path, color='green')  # Plot the ALNS optimized path in green
plot.plot_obstacles(X, all_obstacles)  # Now this line should work
plot.plot_start(X, x_init)
plot.plot_goal(X, x_intermediate, color="pink")
plot.plot_goal(X, x_second_goal, color="blue")
plot.plot_goal(X, x_final_goal, color="green")
plot.draw(auto_open=True)
