# -*- coding:utf-8 -*-
# Train Sarsa in Sokoban environment
import math, os, time, sys
import pdb
import numpy as np
import random, gym
from gym.wrappers import Monitor
from agent import SarsaAgent, SarsaAgent_sokoban
import gym_sokoban
##### START CODING HERE #####
# This code block is optional. You can import other libraries or define your utility functions if necessary.
from matplotlib import pyplot as plt
##### END CODING HERE #####


# construct the environment
env = gym.make('Sokoban-hw2-v0')
# get the size of action space 
num_actions = env.action_space.n
all_actions = np.arange(num_actions)
# set random seed and make the result reproducible
RANDOM_SEED = 0
env.seed(RANDOM_SEED)
random.seed(RANDOM_SEED) 
np.random.seed(RANDOM_SEED) 


####### START CODING HERE #######

# construct the intelligent agent.
agent = SarsaAgent_sokoban(all_actions)
reward_list = []
# start training
for episode in range(1000):
    episode_reward = 0
    s = env.reset()
    # render env. You can comment all render() to turn off the GUI to accelerate training.
    #env.render()
    a = agent.choose_action(s)
    # agent interacts with the environment
    for iter in range(500):
        s_, r, isdone, info = env.step(a)
        a_ = agent.choose_action(s_)
        #env.render()
        episode_reward += r
        print(f"{s} {a} {s_} {r} {isdone}")
        agent.learn(s, a, r, s_, a_, episode)
        s = s_
        a = a_
        if isdone:
            #time.sleep(0.5)
            break
    agent.epsilon_decay(episode)
    print('episode:', episode, 'episode_reward:', episode_reward, 'epsilon:', agent.epsilon)  
    reward_list.append(episode_reward)

print('\ntraining over\n')   
x = [i for i in range(1000)]
plt.scatter(x, reward_list, s = 0.5)
plt.show()
# close the render window after training.
env.close()

####### START CODING HERE #######





