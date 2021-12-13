# -*- coding:utf-8 -*-
# Train Q-Learning in cliff-walking environment
import math, os, time, sys
import numpy as np
import random
import gym
from gym_gridworld import CliffWalk
from agent import QLearningAgent
##### START CODING HERE #####
# This code block is optional. You can import other libraries or define your utility functions if necessary.
from matplotlib import pyplot as plt

def state_to_xy(s):
    x = s % 12
    y = int((s - x) / 12)
    return x,y

##### END CODING HERE #####

# construct the environment
env = CliffWalk()
# get the size of action space 
num_actions = env.action_space.n
all_actions = np.arange(num_actions)
# set random seed and make the result reproducible
RANDOM_SEED = 0
env.seed(RANDOM_SEED)
random.seed(RANDOM_SEED) 
np.random.seed(RANDOM_SEED) 

##### START CODING HERE #####

# construct the intelligent agent.
agent = QLearningAgent(all_actions)
reward_list = []
# start training
for episode in range(1000):
    # record the reward in an episode
    episode_reward = 0
    # reset env
    s = env.reset()
    # render env. You can comment all render() to turn off the GUI to accelerate training.
    #env.render()
    # agent interacts with the environment
    for iter in range(500):
        # choose an action
        x, y = state_to_xy(s)
        a = agent.choose_action(x, y)
        s_, r, isdone, info = env.step(a)
        x_next, y_next = state_to_xy(s_)
        a_next = agent.choose_action_from_qmax(x_next, y_next)
        #env.render()
        # update the episode reward
        episode_reward += r
        print(f"{s} {a} {s_} {r} {isdone}")
        # agent learns from experience
        agent.learn(x, y, a, r, x_next, y_next, a_next)
        s = s_
        if isdone:
            #time.sleep(0.2)
            break
    agent.epsilon_decay(episode)
    print('episode:', episode, 'episode_reward:', episode_reward, 'epsilon:', agent.epsilon)
    reward_list.append(episode_reward)
print('\ntraining over\n')
#print(reward_list)
x = [i for i in range(1000)]
plt.scatter(x, reward_list, s = 0.5)
plt.show()

# close the render window after training.
env.close()

##### START CODING HERE #####


