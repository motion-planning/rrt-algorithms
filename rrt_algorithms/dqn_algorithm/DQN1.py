import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
from collections import deque
import random
import sys
import uuid
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
        model.add(Dense(24, input_dim=self.state_size, activation='relu'))
        model.add(Dense(24, activation='relu'))
        model.add(Dense(self.action_size, activation='linear'))
        model.compile(loss='mse', optimizer=Adam(learning_rate=self.learning_rate))
        return model

    def act(self, state):
        """Selects an action using an epsilon-greedy strategy."""
        state = np.reshape(state, [1, self.state_size])
        if np.random.rand() <= self.epsilon:
            return random.randrange(self.action_size)
        act_values = self.model.predict(state)
        return np.argmax(act_values[0])



    def remember(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))





    def replay(self, batch_size):
        if len(self.memory) < batch_size:
            return
        
        minibatch = random.sample(self.memory, batch_size)
        for state, action, reward, next_state, done in minibatch:
            target = reward
            if not done:
                next_state = np.reshape(next_state, [1, self.state_size])  # Reshape next_state
                target = (reward + self.gamma * np.amax(self.model.predict(next_state)[0]))
            state = np.reshape(state, [1, self.state_size])  # Reshape state
            target_f = self.model.predict(state)
            target_f[0][action] = target
            self.model.fit(state, target_f, epochs=1, verbose=0)
        
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay

class DroneEnv:
    def __init__(self, search_space, start_position, goal_position, obstacles, dwa_params):
        self.search_space = search_space
        self.start_position = start_position
        self.goal_position = goal_position
        self.obstacles = obstacles
        self.dwa_params = dwa_params
        self.state_size = 5  # Make sure this matches the state size
        self.action_size = 9  # Assuming 3 velocity changes and 3 yaw changes
        self.agent = DQNAgent(self.state_size, self.action_size)  # Ensure the DQNAgent is correctly initialized
        self.goal_threshold = 1.0  # Set the threshold for considering the goal as "reached"
        self.reset()

    def reset(self):
        self.current_state = np.array(list(self.start_position) + [0.0, 0.0])  # [x, y, z, velocity, yaw_rate]
        return self.current_state


    def step(self, action):
        velocity_change, yaw_change = self.map_action_to_control(action)
        updated_state, reward, done = self.perform_action(velocity_change, yaw_change)
    
    # Check if the drone is close enough to the goal to consider the episode done
        if np.linalg.norm(updated_state[:3] - np.array(self.goal_position[:3])) < 1.0:
            done = True  # Set the done flag when the goal is reached
    
        return updated_state, reward, done


    def map_action_to_control(self, action):
        """
        Map an action index to a specific velocity change and yaw change.
        """
        velocity_change = [-0.1, 0.0, 0.1][action % 3]  # Modulo 3 to cycle through velocity options
        yaw_change = [-0.1, 0.0, 0.1][(action // 3) % 3]  # Integer division by 3 and modulo 3 for yaw options
        
        return velocity_change, yaw_change

   
    def perform_action(self, velocity_change, yaw_change):
        # Use the current state (already stored in the class)
        state = self.current_state  # Access the current state
        new_velocity = state[3] + velocity_change
        new_yaw_rate = state[4] + yaw_change

        # Update the state
        new_state = np.array([state[0], state[1], state[2], new_velocity, new_yaw_rate])

        # Calculate reward based on the new state
        reward = self.compute_reward(new_state)

        # Check if the goal is reached or there was a collision
        done = self.is_goal_reached(new_state)

        return new_state, reward, done


    def compute_reward(self, state):
        # Calculate the distance to the goal
        distance_to_goal = np.linalg.norm(state[:3] - self.goal_position[:3])

        # Reward for reaching the goal
        if distance_to_goal < self.goal_threshold:
            return 1000  # Large reward for reaching the goal

        # Penalize based on the distance to the goal
        reward = -distance_to_goal

        # Optionally, add additional penalties for collisions or other constraints

        return reward
    
    def is_goal_reached(self, state):
        # Check if the drone is within the threshold distance to the goal
        return np.linalg.norm(state[:3] - self.goal_position[:3]) < self.goal_threshold





    # Rest of the code remains the same

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
            vec1 = np.array(path[i][:3]) - np.array(path[i-1][:3])
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
        # Map the action to a velocity and yaw rate change
        velocity_change = [-0.1, 0.0, 0.1][action % 3]  # Modulo 3 for velocity options
        yaw_change = [-0.1, 0.0, 0.1][(action // 3) % 3]  # Integer division for yaw options
        return velocity_change, yaw_change

    def train(self, episodes, batch_size=64):
        for episode in range(episodes):
            state = self.reset()
            total_reward = 0
            done = False

            while not done:
                action = self.agent.act(state)  # Ensure `act` is properly called here
                next_state, reward, done = self.step(action)
                total_reward += reward
                self.agent.remember(state, action, reward, next_state, done)
                state = next_state

            # Replay experiences with the set batch size
                self.agent.replay(batch_size)

            print(f"Episode: {episode + 1}/{episodes}, Total Reward: {total_reward}")

