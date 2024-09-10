import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyController:
    def __init__(self):
        # Define fuzzy variables
        self.dist_to_goal = ctrl.Antecedent(np.arange(0, 101, 1), 'dist_to_goal')
        self.dist_to_obstacle = ctrl.Antecedent(np.arange(0, 101, 1), 'dist_to_obstacle')
        
        self.to_goal_gain = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'to_goal_gain')
        self.speed_gain = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'speed_gain')
        self.obstacle_gain = ctrl.Consequent(np.arange(0, 2.1, 0.1), 'obstacle_gain')

        # Membership functions for distance to goal
        self.dist_to_goal['close'] = fuzz.trimf(self.dist_to_goal.universe, [0, 0, 30])
        self.dist_to_goal['medium'] = fuzz.trimf(self.dist_to_goal.universe, [20, 50, 80])
        self.dist_to_goal['far'] = fuzz.trimf(self.dist_to_goal.universe, [70, 100, 100])

        # Membership functions for distance to obstacles
        self.dist_to_obstacle['close'] = fuzz.trimf(self.dist_to_obstacle.universe, [0, 0, 30])
        self.dist_to_obstacle['medium'] = fuzz.trimf(self.dist_to_obstacle.universe, [20, 50, 80])
        self.dist_to_obstacle['far'] = fuzz.trimf(self.dist_to_obstacle.universe, [70, 100, 100])

        # Membership functions for gain adjustments
        self.to_goal_gain['low'] = fuzz.trimf(self.to_goal_gain.universe, [0, 0, 1])
        self.to_goal_gain['medium'] = fuzz.trimf(self.to_goal_gain.universe, [0.5, 1.0, 1.5])
        self.to_goal_gain['high'] = fuzz.trimf(self.to_goal_gain.universe, [1.0, 2.0, 2.0])

        self.speed_gain['low'] = fuzz.trimf(self.speed_gain.universe, [0, 0, 1])
        self.speed_gain['medium'] = fuzz.trimf(self.speed_gain.universe, [0.5, 1.0, 1.5])
        self.speed_gain['high'] = fuzz.trimf(self.speed_gain.universe, [1.0, 2.0, 2.0])

        self.obstacle_gain['low'] = fuzz.trimf(self.obstacle_gain.universe, [0, 0, 1])
        self.obstacle_gain['medium'] = fuzz.trimf(self.obstacle_gain.universe, [0.5, 1.0, 1.5])
        self.obstacle_gain['high'] = fuzz.trimf(self.obstacle_gain.universe, [1.0, 2.0, 2.0])

        # Define fuzzy rules
        self.rule1 = ctrl.Rule(self.dist_to_goal['close'] & self.dist_to_obstacle['close'], 
                               [self.to_goal_gain['low'], self.speed_gain['medium'], self.obstacle_gain['high']])
        self.rule2 = ctrl.Rule(self.dist_to_goal['medium'] & self.dist_to_obstacle['medium'], 
                               [self.to_goal_gain['medium'], self.speed_gain['medium'], self.obstacle_gain['medium']])
        self.rule3 = ctrl.Rule(self.dist_to_goal['far'] & self.dist_to_obstacle['far'], 
                               [self.to_goal_gain['high'], self.speed_gain['high'], self.obstacle_gain['low']])
        self.rule4 = ctrl.Rule(self.dist_to_goal['far'] & self.dist_to_obstacle['close'], 
                               [self.to_goal_gain['medium'], self.speed_gain['low'], self.obstacle_gain['high']])
        self.rule5 = ctrl.Rule(self.dist_to_goal['close'] & self.dist_to_obstacle['far'], 
                               [self.to_goal_gain['high'], self.speed_gain['high'], self.obstacle_gain['low']])

        self.control_system = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5])
        self.simulation = ctrl.ControlSystemSimulation(self.control_system)

    def compute_gains(self, dist_to_goal, dist_to_obstacle):
        self.simulation.input['dist_to_goal'] = dist_to_goal
        self.simulation.input['dist_to_obstacle'] = dist_to_obstacle
        self.simulation.compute()
        return (self.simulation.output['to_goal_gain'], 
                self.simulation.output['speed_gain'], 
                self.simulation.output['obstacle_gain'])


