# -*- coding:utf-8 -*-
import math, os, time, sys
import numpy as np
import gym
##### START CODING HERE #####
# This code block is optional. You can import other libraries or define your utility functions if necessary.
from scipy.stats import beta
##### END CODING HERE #####

"""
Instruction: 
Currently, the following agents are `random` policy.
You need to implement the Q-learning agent, Sarsa agent and Dyna-Q agent in this file.
"""

# ------------------------------------------------------------------------------------------- #

"""TODO: Implement your Sarsa agent here"""
class SarsaAgent(object):
    ##### START CODING HERE #####
    def __init__(self, all_actions):
        """initialize the agent. Maybe more function inputs are needed."""
        self.all_actions = all_actions
        self.epsilon = 1
        self.gamma = 0.95
        self.learning_rate = 0.1
        self.Q = np.zeros((12, 4, 4))

    def choose_action(self, x, y):
        """choose action with epsilon-greedy algorithm."""
        if np.random.rand() < self.epsilon:
            action = np.random.choice(self.all_actions)
        else:
            action = self.choose_action_from_qmax(x, y)
        return action
        
    def learn(self, x, y, a, reward, x_next, y_next, a_next):
        """learn from experience"""
        self.Q[x][y][a] += self.learning_rate * (reward + self.gamma * self.Q[x_next][y_next][a_next] - self.Q[x][y][a])
        print("[INFO] The learning process complete. (ﾉ｀⊿´)ﾉ")
        return True

    def choose_action_from_qmax(self, x, y):
        max_q = np.max(self.Q[x][y])
        max_actions = []
        for i in range(4):
            if self.Q[x][y][i] == max_q:
                max_actions.append(i)
        action = np.random.choice(max_actions)
        return action

    def epsilon_decay(self, episode):
        if episode < 200:
            self.epsilon = 1 - 0.0015 * episode
        elif episode < 400:
            self.epsilon = 0.5 - 0.0005 * episode
        else:
            self.epsilon = 0.1 - 0.0001 * episode
        '''
        if episode < 500:
            self.epsilon = 1 - 0.001 * episode
        else:
            self.epsilon = 0.4 - 0.0004 * episode
        '''
    
    def your_function(self, params):
        """You can add other functions as you wish."""
        do_something = True
        return None

class SarsaAgent_sokoban(object):
    ##### START CODING HERE #####
    def __init__(self, all_actions):
        """initialize the agent. Maybe more function inputs are needed."""
        self.all_actions = all_actions
        self.epsilon = 1
        self.gamma = 0.9
        self.learning_rate = 0.1
        self.Q = np.zeros((5, 5, 5, 5, 5, 5, 4))

    def choose_action(self, s):
        """choose action with epsilon-greedy algorithm."""
        if np.random.rand() < self.epsilon:
            action = np.random.choice(self.all_actions)
        else:
            action = self.choose_action_from_qmax(s)
        return action
    
    def learn(self, s, a, reward, s_next, a_next, episode):
        """learn from experience"""
        self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1][a] += self.learning_rate * (reward + self.gamma * self.Q[s_next[0]-1][s_next[1]-1][s_next[2]-1][s_next[3]-1][s_next[4]-1][s_next[5]-1][a_next] - self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1][a])
        print("[INFO] The learning process complete. (ﾉ｀⊿´)ﾉ")
        return True
    
    def choose_action_from_qmax(self, s):
        max_q = np.max(self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1])
        max_actions = []
        for i in range(4):
            if self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1][i] == max_q:
                max_actions.append(i)
        action = np.random.choice(max_actions)
        return action

    def epsilon_decay(self, i):
        self.epsilon = self.epsilon * 0.1
        #self.epsilon = 1 - math.log(i+1, 1000)
        #if episode < 400:
        #    self.epsilon = 1 - 0.001 * episode
        #elif episode < 600:
        #    self.epsilon = 0.4 - 0.0004 * episode
        #else:
        #    self.epsilon = 0.15 - 0.0001 * episode

    ##### END CODING HERE #####

# ------------------------------------------------------------------------------------------- #

"""TODO: Implement your Q-Learning agent here"""
class QLearningAgent(object):
    ##### START CODING HERE #####
    def __init__(self, all_actions):
        """initialize the agent. Maybe more function inputs are needed."""
        self.all_actions = all_actions
        self.epsilon = 1
        self.gamma = 1
        self.learning_rate = 0.1
        self.Q = np.zeros((12, 4, 4))

    def choose_action(self, x, y):
        """choose action with epsilon-greedy algorithm."""
        if np.random.rand() < self.epsilon:
            action = np.random.choice(self.all_actions)
        else:
            action = self.choose_action_from_qmax(x, y)
        return action
    
    def learn(self, x, y, a, reward, x_next, y_next, a_next):
        """learn from experience"""
        self.Q[x][y][a] += self.learning_rate * (reward + self.gamma * self.Q[x_next][y_next][a_next] - self.Q[x][y][a])
        #time.sleep(0.2)
        print("[INFO] The learning process complete. (ﾉ｀⊿´)ﾉ")
        return True
    
    def choose_action_from_qmax(self, x, y):
        max_q = np.max(self.Q[x][y])
        max_actions = []
        for i in range(4):
            if self.Q[x][y][i] == max_q:
                max_actions.append(i)
        action = np.random.choice(max_actions)
        return action

    def epsilon_decay(self, episode):
        if episode < 200:
            self.epsilon = 1 - 0.0015 * episode
        elif episode < 400:
            self.epsilon = 0.5 - 0.0005 * episode
        else:
            self.epsilon = 0.1 - 0.0001 * episode

    def your_function(self, params):
        """You can add other functions as you wish."""
        do_something = True
        return None

class QLearningAgent_sokoban(object):
    ##### START CODING HERE #####
    def __init__(self, all_actions):
        """initialize the agent. Maybe more function inputs are needed."""
        self.all_actions = all_actions
        self.epsilon = 1
        self.gamma = 0.95
        self.learning_rate = 0.1
        self.Q = np.zeros((5, 5, 5, 5, 5, 5, 4))

    def choose_action(self, s):
        """choose action with epsilon-greedy algorithm."""
        if np.random.rand() < self.epsilon:
            action = np.random.choice(self.all_actions)
        else:
            action = self.choose_action_from_qmax(s)
        return action
    
    def learn(self, s, a, reward, s_next, a_next, episode):
        """learn from experience"""
        self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1][a] += self.learning_rate * (reward + self.gamma * self.Q[s_next[0]-1][s_next[1]-1][s_next[2]-1][s_next[3]-1][s_next[4]-1][s_next[5]-1][a_next] - self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1][a])
        if episode > 970:
            time.sleep(0.1)
        print("[INFO] The learning process complete. (ﾉ｀⊿´)ﾉ")
        return True
    
    def choose_action_from_qmax(self, s):
        max_q = np.max(self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1])
        max_actions = []
        for i in range(4):
            if self.Q[s[0]-1][s[1]-1][s[2]-1][s[3]-1][s[4]-1][s[5]-1][i] == max_q:
                max_actions.append(i)
        action = np.random.choice(max_actions)
        return action

    def epsilon_decay(self, episode):
        self.epsilon = 0.5*(1 - math.log(episode+1, 1000))
        #if episode < 400:
        #    self.epsilon = 1 - 0.001 * episode
        #elif episode < 600:
        #    self.epsilon = 0.4 - 0.0004 * episode
        #else:
        #    self.epsilon = 0.15 - 0.0001 * episode

    ##### END CODING HERE #####

# ------------------------------------------------------------------------------------------- #

"""TODO: Implement your Dyna-Q agent here"""
class DynaQAgent(object):
    ##### START CODING HERE #####
    def __init__(self, all_actions):
        """initialize the agent. Maybe more function inputs are needed."""
        self.all_actions = all_actions
        self.epsilon = 1.0
        self.gamma = 0.95
        self.learning_rate = 0.1
        self.Q = {}
        self.model = {}
        
    def choose_action(self, observation):
        """choose action with epsilon-greedy algorithm."""
        if np.random.rand() < self.epsilon:
            if str(observation) not in self.Q:
                self.Q[str(observation)] = np.zeros(4)
            action = np.random.choice(self.all_actions)
        else:
            action = self.choose_action_from_qmax(observation)
        return action
    
    def learn(self, observation, a, r, observation_next, a_next):
        """learn from experience"""
        self.Q[str(observation)][a] += self.learning_rate * (r + self.gamma * self.Q[str(observation_next)][a_next] - self.Q[str(observation)][a])
        #print("[INFO] The learning process complete. (ﾉ｀⊿´)ﾉ")
        return True
    
    def choose_action_from_qmax(self, observation):
        if str(observation) not in self.Q:
            self.Q[str(observation)] = np.zeros(4)
        max_q = max(self.Q[str(observation)])
        max_actions = []
        for i in range(4):
            if self.Q[str(observation)][i] == max_q:
                max_actions.append(i)
        return np.random.choice(max_actions)

    def epsilon_decay(self, episode):
        self.epsilon = 1 - math.log(episode+1, 1000)


    def your_function(self, params):
        """You can add other functions as you wish."""
        do_something = True
        return None

    ##### END CODING HERE #####

# ------------------------------------------------------------------------------------------- #
##### START CODING HERE #####
"""TODO: (optional) Implement RL agent(s) with other exploration methods you have found"""
class RLAgentWithOtherExploration(object):
    """initialize the agent"""
    def __init__(self, all_actions):
        self.all_actions = all_actions
        self.epsilon = 1.0
        self.chosen_count = {}
        self.esitimated_rewards = {}
        self.Q = {}
        self.gamma = 0.9
        self.learning_rate = 0.1
      
    def calculate_delta(self, T, observation, action):
        if not self.chosen_count.__contains__(str(observation)):
            self.chosen_count[str(observation)] = [0,0,0,0]
        if self.chosen_count[str(observation)][action] ==0:
            return 1
        else:
            return np.sqrt(2*np.log(T+1)/self.chosen_count[str(observation)][action]) 
    
    def choose_action(self, observation, episode):
        """choose action witch other exploration algorithms."""
        if not self.esitimated_rewards.__contains__(str(observation)):
            self.esitimated_rewards[str(observation)] = [0,0,0,0]
        upper_bound_probs = [self.esitimated_rewards[str(observation)][action]+ \
             self.calculate_delta(episode, observation, action) for action in self.all_actions ]
        max_prob = np.max(upper_bound_probs)
        max_actions = []
        for i in range(4):
            if upper_bound_probs[i] == max_prob:
                max_actions.append(i)
        action = np.random.choice(max_actions)
        return action
    
    def update(self, observation, action, reward):
        if not self.chosen_count.__contains__(str(observation)):
            self.chosen_count[str(observation)] = [0,0,0,0]
        self.esitimated_rewards[str(observation)][action]*= self.chosen_count[str(observation)][action]
        self.esitimated_rewards[str(observation)][action]+= reward
        self.chosen_count[str(observation)][action] += 1
        self.esitimated_rewards[str(observation)][action]/= self.chosen_count[str(observation)][action]
        
    def learn_Q(self, observation, a, r, observation_next):
        """learn from experience"""
        if str(observation) not in self.Q:
            self.Q[str(observation)] = np.zeros(4)
        if str(observation_next) not in self.Q:
            self.Q[str(observation_next)] = np.zeros(4)
        actual_reward = r + self.gamma * max(self.Q[str(observation_next)])
        self.Q[str(observation)][a] += self.learning_rate * (actual_reward - self.Q[str(observation)][a])
        return actual_reward

    def learn(self,episode,action, observation, reward, observation_next):
        """learn from experience"""
        actual_reward = self.learn_Q(observation, action, reward, observation_next)
        self.update(observation, action, actual_reward)
        print("[INFO] The learning process complete. (ﾉ｀⊿´)ﾉ")
        return True
##### END CODING HERE #####
# ------------------------------------------------------------------------------------------- #