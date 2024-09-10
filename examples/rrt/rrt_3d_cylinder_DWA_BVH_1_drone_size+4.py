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
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
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
n = 10  # Number of random obstacles
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
q = 500  # Extension length
r = 1
max_samples = 4000  # Maximum samples
prc = 0.1  # Probability of choosing goal as the next point

# DWA parameters
dwa_params = {
    'max_speed': 2.0,
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

# Functions for computing metrics
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

# ALNS functions
def alns_optimize_path(path, X, all_obstacles, max_iterations=100):
    neighborhoods = [segment_change, detour_addition, direct_connection]
    neighborhood_scores = {segment_change: 1.0, detour_addition: 1.0, direct_connection: 1.0}
    current_path = path.copy()

    for _ in range(max_iterations):
        neighborhood = random.choices(neighborhoods, weights=neighborhood_scores.values())[0]
        new_path = neighborhood(current_path, X, all_obstacles)

        if path_length(new_path) < path_length(current_path):
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
        return path

    new_path = path.copy()
    i = random.randint(0, len(path) - 3)
    j = random.randint(i + 2, len(path) - 1)

    x_a = path[i][:3]
    x_b = path[j][:3]

    if X.collision_free(x_a, x_b, r):
        new_path = new_path[:i + 1] + new_path[j:]

    return new_path

# Continuous Learning Loop Parameters
episodes = 10  # Number of training episodes
optimize_every = 10  # Frequency of re-optimization and re-training
# Initialize DQN Agent
env = DroneEnv(X, x_init, x_final_goal, all_obstacles, dwa_params)
try:
    agent = DQNAgent(env.state_size, env.action_size)
    agent.model = load_model("dqn_model.keras")
    print("Loaded existing DQN model.")
except:
    agent = DQNAgent(env.state_size, env.action_size)
    print("Initialized new DQN model.")

# Lists to store paths for animation
paths_to_animate = []

# Training Loop
for episode in range(episodes):
    print(f"Episode: {episode + 1}/{episodes}")

    state = env.reset()
    done = False
    episode_path = []

    # RRT Pathfinding to the goals
    print("RRT pathfinding...")
    rrt = RRT(X, q, x_init, x_intermediate, max_samples, r, prc)
    path1 = rrt.rrt_search()

    if path1 is not None:
        rrt = RRT(X, q, x_intermediate, x_second_goal, max_samples, r, prc)
        path2 = rrt.rrt_search()

        if path2 is not None:
            rrt = RRT(X, q, x_second_goal, x_final_goal, max_samples, r, prc)
            path3 = rrt.rrt_search()

            if path3 is not None:
                path = path1 + path2[1:] + path3[1:]
            else:
                path = path1 + path2[1:]
        else:
            path = path1
    else:
        path = None

    if path is not None:
        print("Optimizing path with DWA...")
        dwa = DWA(dwa_params)
        optimized_path = []

        for i in range(len(path) - 1):
            start_point = path[i] + (0.0, 0.0)
            end_point = path[i + 1]
            local_path = dwa.plan(start_point, end_point, X, all_obstacles)
            optimized_path.extend(local_path)

        print("Optimizing path with ALNS...")
        alns_optimized_path = alns_optimize_path(optimized_path, X, all_obstacles, max_iterations=100)

        paths_to_animate.append(alns_optimized_path)

        for idx, point in enumerate(alns_optimized_path):
            action = agent.act(state)
            next_state, reward, done = env.step(action)
            agent.remember(state, action, reward, next_state, done)
            episode_path.append(next_state)
            state = next_state

        if (episode + 1) % optimize_every == 0:
            print("Re-training the DQN model...")
            agent.replay(32)
            agent.model.save("dqn_model.keras")

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
    else:
        print("No path found, skipping episode.")

# Plotting and Animation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Function to draw cylindrical obstacles
def draw_cylinder(ax, center_x, center_y, z_min, z_max, radius, color='red', alpha=0.5):
    z = np.linspace(z_min, z_max, 100)
    theta = np.linspace(0, 2*np.pi, 100)
    theta_grid, z_grid = np.meshgrid(theta, z)
    x_grid = radius * np.cos(theta_grid) + center_x
    y_grid = radius * np.sin(theta_grid) + center_y
    ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=alpha)

# Function to update the plot
def update(num):
    ax.clear()
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 100)
    ax.set_zlim(0, 100)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f"Path Optimization - Episode {num + 1}")

    path = paths_to_animate[num]
    x_vals = [point[0] for point in path]
    y_vals = [point[1] for point in path]
    z_vals = [point[2] for point in path]
    ax.plot(x_vals, y_vals, z_vals, color='green')

    for obs in all_obstacles:
        draw_cylinder(ax, obs[0], obs[1], obs[2], obs[3], obs[4])

    ax.scatter(x_init[0], x_init[1], x_init[2], c='blue', marker='o', label="Start")
    ax.scatter(x_intermediate[0], x_intermediate[1], x_intermediate[2], c='pink', marker='^', label="Intermediate Goal")
    ax.scatter(x_second_goal[0], x_second_goal[1], x_second_goal[2], c='purple', marker='s', label="Second Goal")
    ax.scatter(x_final_goal[0], x_final_goal[1], x_final_goal[2], c='yellow', marker='*', label="Final Goal")

ani = FuncAnimation(fig, update, frames=len(paths_to_animate), interval=10000, repeat=False)
plt.show()
