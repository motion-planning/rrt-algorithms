#generate random poisitions:
import numpy as np
import time
from rtree import index
import sys
import uuid
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import random
from tensorflow.keras.models import load_model
import matplotlib.pyplot as plt
import matplotlib.animation as animation

sys.path.append('/home/dell/rrt-algorithms')

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space2 import SearchSpace
from rrt_algorithms.utilities.plotting2 import Plot
from rrt_algorithms.dwa_algorithm.DWA3 import DWA, FuzzyController
from rrt_algorithms.dqn_algorithm.DQN import DQNAgent
from rrt_algorithms.dqn_algorithm.DQN import DroneEnv
from rrt_algorithms.utilities.obstacle_generation2 import generate_random_cylindrical_obstacles 

# Define the search space dimensions
X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])
X = SearchSpace(X_dimensions)

# Helper function to generate random positions
def generate_random_position(dimensions):
    x_min, x_max = dimensions[0]
    y_min, y_max = dimensions[1]
    z_min, z_max = dimensions[2]
    return (
        random.uniform(x_min, x_max),
        random.uniform(y_min, y_max),
        random.uniform(z_min, z_max)
    )

# Initial and goal positions (Randomized)
x_init = generate_random_position(X_dimensions)  # Random starting position
x_intermediate = generate_random_position(X_dimensions)  # Random intermediate goal
x_second_goal = generate_random_position(X_dimensions)  # Random second goal
x_final_goal = generate_random_position(X_dimensions)  # Random final goal

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
max_samples = 4024
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
    neighborhoods = [segment_change, detour_addition, direct_connection]
    neighborhood_scores = {segment_change: 1.0, detour_addition: 1.0, direct_connection: 1.0}
    current_path = path.copy()

    for _ in range(max_iterations):
        neighborhood = random.choices(neighborhoods, weights=neighborhood_scores.values())[0]
        new_path = neighborhood(current_path, X, all_obstacles)

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
    if len(path) < 3:
        return path  # Can't directly connect if there aren't enough points

    new_path = path.copy()
    i = random.randint(0, len(path) - 3)
    j = random.randint(i + 2, len(path) - 1)

    x_a = path[i][:3]
    x_b = path[j][:3]

    if X.collision_free(x_a, x_b, r):
        new_path = new_path[:i + 1] + new_path[j:]

    return new_path

def find_closest_point(path, goal):
    distances = [np.linalg.norm(np.array(p[:3]) - np.array(goal)) for p in path]
    return np.argmin(distances)


def is_inside_obstacle(point, obstacles):
    """
    Check if a point is inside any obstacle.
    Assumes obstacles are represented as (x_center, y_center, z_min, z_max, radius).
    """
    for obs in obstacles:
        x_center, y_center, z_min, z_max, radius = obs
        distance = np.linalg.norm(np.array(point[:2]) - np.array([x_center, y_center]))
        if distance < radius and z_min <= point[2] <= z_max:
            return True
    return False

# Function to try RRT to a goal and return the path or None
def try_rrt_to_goal(X, q, start, goal, max_samples, r, prc, obstacles):
    if is_inside_obstacle(goal, obstacles):
        print(f"Destination {goal} is inside an obstacle and not achievable.")
        return None
    else:
        rrt = RRT(X, q, start, goal, max_samples, r, prc)
        return rrt.rrt_search()

# RRT pathfinding to the first intermediate goal
print("RRT to first intermediate goal...")
start_time = time.time()
path1 = try_rrt_to_goal(X, q, x_init, x_intermediate, max_samples, r, prc, all_obstacles)
rrt_time = time.time() - start_time

if path1 is not None:
    print("RRT to second intermediate goal...")
    path2 = try_rrt_to_goal(X, q, x_intermediate, x_second_goal, max_samples, r, prc, all_obstacles)

    if path2 is not None:
        print("RRT to final goal...")
        path3 = try_rrt_to_goal(X, q, x_second_goal, x_final_goal, max_samples, r, prc, all_obstacles)

        # Combine all paths
        if path3 is not None:
            path = path1 + path2[1:] + path3[1:]
        else:
            path = path1 + path2[1:]
    else:
        path = path1
else:
    path = None

# Before calculating the path length, check if the path is valid (not None)
if path is not None:
    # Calculate and print path length
    rrt_path_length = path_length(path)
    print(f"RRT Path Length: {rrt_path_length}")
else:
    print("Error: RRT failed to find a valid path. Path length calculation skipped.")


# Apply DWA for local optimization along the RRT path
if path is not None:
    dwa = DWA(dwa_params)
    optimized_path = []

    start_time = time.time()
    for i in range(len(path) - 1):
        start_point = path[i] + (0.0, 0.0)
        end_point = path[i + 1]

        local_path = dwa.plan(start_point, end_point, X, all_obstacles)
        optimized_path.extend(local_path)
    dwa_time = time.time() - start_time

    # Apply ALNS optimization to each segment of the path
    print("Applying ALNS optimization...")
    alns_optimized_path = []
    
    index_intermediate = find_closest_point(optimized_path, x_intermediate)
    index_second_goal = find_closest_point(optimized_path, x_second_goal)
    
    segment1 = optimized_path[:index_intermediate + 1]
    alns_optimized_segment1 = alns_optimize_path(segment1, X, all_obstacles, max_iterations=100)
    
    segment2 = optimized_path[index_intermediate:index_second_goal + 1]
    alns_optimized_segment2 = alns_optimize_path(segment2, X, all_obstacles, max_iterations=100)
    
    segment3 = optimized_path[index_second_goal:]
    alns_optimized_segment3 = alns_optimize_path(segment3, X, all_obstacles, max_iterations=100)
    
    alns_optimized_path = alns_optimized_segment1 + alns_optimized_segment2[1:] + alns_optimized_segment3[1:]

    def validate_and_correct_path(path):
        corrected_path = []
        for idx, point in enumerate(path):
            if isinstance(point, (list, tuple)) and len(point) >= 2:
                if isinstance(point[0], (list, tuple)):
                    point = [item for sublist in point for item in sublist]
                corrected_path.append(point[:3])
            else:
                raise ValueError(f"Point {idx} in path is incorrectly formatted: {point}")
        return corrected_path

    optimized_path = validate_and_correct_path(optimized_path)
    alns_optimized_path = validate_and_correct_path(alns_optimized_path)

# Continue the rest of your code as before...



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
env.train(episodes=50)
# After training, use the learned policy in your DWA

# Initialize the DroneEnv environment for path optimization
state = env.reset()
done = False
ml_path = []
agent = DQNAgent(env.state_size, env.action_size)

while not done:
    action = agent.act(state)
    next_state, _, done = env.step(action)
    ml_path.append(next_state)
    state = next_state

# Convert ml_path to a numpy array
ml_path = np.array(ml_path)

# Check if ml_path has valid data
if ml_path.size == 0:
    print("ML path is empty. Ensure the DQN agent is working correctly.")
else:
    print("ML path:", ml_path)

# Save the trained DQN model
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

# Ensure the final paths are properly flattened and structured
def flatten_path_points(path):
    cleaned_path = []
    for idx, point in enumerate(path):
        if isinstance(point, np.ndarray):
            flat_point = point.flatten().tolist()
            if len(flat_point) >= 3:
                cleaned_path.append(flat_point[:3])  # Take only the first three elements (x, y, z)
            else:
                raise ValueError(f"Point {idx} in optimized_path is incorrectly formatted: {flat_point}")
        elif isinstance(point, (list, tuple)) and len(point) >= 3:
            cleaned_path.append(point[:3])  # Handle lists or tuples directly
        else:
            raise ValueError(f"Point {idx} in optimized_path is not correctly formatted: {point}")
    return np.array(cleaned_path)

# Apply the flattening function to optimized_path before plotting
try:
    optimized_path = flatten_path_points(optimized_path)
    ml_path = flatten_path_points(ml_path)
except ValueError as e:
    print(f"Error encountered during flattening: {e}")

# Plotting and Animation

# First figure: RRT, DWA, ALNS paths with ML path
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(X_dimensions[0])
ax.set_ylim(X_dimensions[1])
ax.set_zlim(X_dimensions[2])

# Plot the RRT, DWA, and ALNS paths as dashed lines
ax.plot(*zip(*path), linestyle='--', color='red', label="RRT Path")
ax.plot(*zip(*optimized_path), linestyle='--', color='blue', label="DWA Optimized Path")
ax.plot(*zip(*alns_optimized_path), linestyle='--', color='green', label="ALNS Optimized Path")

# Plot the ML path
ax.plot(*ml_path.T, color='black', label="ML Path")

# Plot static obstacles
for obs in all_obstacles:
    x_center, y_center, z_min, z_max, radius = obs
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(z_min, z_max, 100)
    x = radius * np.outer(np.cos(u), np.ones(len(v))) + x_center
    y = radius * np.outer(np.sin(u), np.ones(len(v))) + y_center
    z = np.outer(np.ones(len(u)), v)
    ax.plot_surface(x, y, z, color='gray', alpha=0.3)

# Plot start and goal points
ax.scatter(*x_init, color='magenta', label="Start")
ax.scatter(*x_intermediate, color='pink', label="Intermediate Goal")
ax.scatter(*x_second_goal, color='cyan', label="Second Goal")
ax.scatter(*x_final_goal, color='green', label="Final Goal")

# Animation of the ML path on the same figure
drone_path_line, = ax.plot([], [], [], color='orange', linewidth=2)

def update_line(num, ml_path, line):
    line.set_data(ml_path[:num, 0], ml_path[:num, 1])
    line.set_3d_properties(ml_path[:num, 2])
    return line,

ani = animation.FuncAnimation(
    fig, update_line, frames=len(ml_path), fargs=(ml_path, drone_path_line), interval=100, blit=True
)

# Add legend and show the figure
ax.legend()
plt.show()


