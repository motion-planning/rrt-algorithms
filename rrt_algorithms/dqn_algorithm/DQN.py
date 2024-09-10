import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
from collections import deque
import random
import sys
import uuid
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
sys.path.append('/home/dell/rrt-algorithms')

from rrt_algorithms.rrt.rrt import RRT
from rrt_algorithms.search_space.search_space2 import SearchSpace
from rrt_algorithms.utilities.plotting2 import Plot
from rrt_algorithms.dwa_algorithm.DWA3 import DWA 
from rrt_algorithms.dwa_algorithm.DWA3 import FuzzyController
from rrt_algorithms.utilities.obstacle_generation2 import generate_random_cylindrical_obstacles 
class DQNAgent:
    def __init__(self, state_size, action_size):
        self.state_size = state_size
        self.action_size = action_size
        self.memory = deque(maxlen=2000)
        self.gamma = 0.95    # discount rate
        self.epsilon = 1.0   # exploration rate
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.learning_rate = 0.001
        self.model = self._build_model()

    def _build_model(self):
        model = Sequential()
        model.add(Dense(24, input_dim=6, activation='relu'))  # Input_dim should be 6
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=Adam(learning_rate=self.learning_rate))
        return model

    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))

    def act(self, state):
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])

    def replay(self, batch_size):
        # Ensure there are enough samples in memory to form a full batch
        if len(self.memory) < batch_size:
            return

        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                target = (reward + self.gamma *
                          np.amax(self.model.predict(next_state)[0]))
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

class DroneEnv:
    def __init__(self, X, start, goal, obstacles, dwa_params):
        self.X = X
        self.start = start
        self.goal = goal
        self.obstacles = obstacles
        self.dwa_params = dwa_params
        self.state_size = 6
        self.action_size = 3
        self.agent = DQNAgent(self.state_size, self.action_size)
        self.reset()

    def reset(self):
        self.state = np.array(self.start + (0.0, 0.0, 0.0))
        return self.state.reshape(1, -1)

    def step(self, action):
        velocity_change, yaw_change = self.map_action_to_control(action)
        next_state, reward, done = self.perform_action(velocity_change, yaw_change)
        return next_state, reward, done

    def train(self, episodes, batch_size=64):  # Set your batch size here
        for episode in range(episodes):
            state = self.reset()
            total_reward = 0
            done = False

            while not done:
                action = self.agent.act(state)
                next_state, reward, done = self.step(action)
                total_reward += reward
                self.agent.remember(state, action, reward, next_state, done)
                state = next_state

                # Replay experiences with the set batch size
                self.agent.replay(batch_size)

            print(f"Episode: {episode + 1}/{episodes}, Total Reward: {total_reward}")

    
    def compute_reward(self, state, path):
    # Distance to the goal
        distance_to_goal = np.linalg.norm(np.array(state[:2]) - np.array(self.goal[:2]))

    # Obstacle clearance
        obstacle_clearance = self.min_obstacle_clearance(path, self.obstacles)

    # Path smoothness (penalizing sharp turns)
        path_smoothness = self.path_smoothness(path)

    # Energy usage (penalizing high energy usage)
        energy_usage = self.compute_energy_usage(path, state[3])  # state[3] is the current velocity

    # Calculate the reward
        reward = -distance_to_goal  # Encourage getting closer to the goal

    # Penalize getting too close to obstacles
        if obstacle_clearance < 1.0:  # 1.0 is a threshold, you can adjust this
            reward -= 100 * (1.0 - obstacle_clearance)

    # Penalize sharp turns in the path
        reward -= 10 * path_smoothness

    # Penalize high energy usage
        reward -= 0.1 * energy_usage
    
    # Small positive reward for staying alive (not crashing)
        reward += 1.0

        return reward


    def perform_action(self, velocity_change, yaw_change):
        state = self.state
        new_velocity = state[3] + velocity_change
        new_yaw_rate = state[4] + yaw_change

    # Ensure the state vector always has 6 elements
        state = np.array([state[0], state[1], state[2], new_velocity, new_yaw_rate, state[5]])  # Adjust the last value as needed

        dwa = DWA(self.dwa_params)
        local_path = dwa.plan(state, self.goal, self.X, self.obstacles)

        if not local_path:
            reward = -100  # Collision penalty
            done = True
            updated_state = state  # Maintain the current state
        else:
        # Ensure the state from local_path[-1] is extended to 6 elements
            updated_state = np.array(local_path[-1])
            if len(updated_state) == 5:
                updated_state = np.append(updated_state, state[5])  # Maintain the last element (e.g., energy usage or other feature)
        
            reward = self.compute_reward(updated_state, local_path)
            done = np.linalg.norm(np.array(updated_state[:3]) - np.array(self.goal[:3])) < 1.0  # Check if the goal is reached

        print("State shape after update:", updated_state.shape)
        return updated_state.reshape(1, -1), reward, done  # This return should be inside the function


# Ensure all other parts of the code that handle the state are consistent with the 6-element structure.
        
    def min_obstacle_clearance(self, path, obstacles):
        min_clearance = float('inf')
        for point in path:
            for obs in obstacles:
                clearance = np.linalg.norm(np.array(point[:2]) - np.array(obs[:2])) - obs[4]
                if clearance < min_clearance:
                    min_clearance = clearance
        return min_clearance
    def path_smoothness(self, path):
        total_curvature = 0.0
        for i in range(1, len(path) - 1):
            vec1 = np.array(path[i][:3]) - np.array(path[i-1][:3])  # Use the first 3 elements if these are the coordinates
            vec2 = np.array(path[i+1][:3]) - np.array(path[i][:3])
            angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))
            total_curvature += angle
        return total_curvature
    
    def compute_energy_usage(self, path, velocity):
        energy = 0.0
        for i in range(1, len(path)):
            distance = np.linalg.norm(np.array(path[i][:3]) - np.array(path[i-1][:3]))
            energy += distance * velocity
        return energy
        


    def map_action_to_control(self, action):
        # Simplified action space
        velocity_change = [-0.1, 0.0, 0.1][action % 3]  # Example action mapping
        yaw_change = 0  # Keep yaw_change static for simplicity
        return velocity_change, yaw_change

