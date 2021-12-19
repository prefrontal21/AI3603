# -*- coding:utf-8 -*-
# Train any RL agent in Sokoban environment with new exploration method.

import math, os, time, sys
import numpy as np
import random, gym
from gym.wrappers import Monitor
from agent import RLAgentWithOtherExploration
import gym_sokoban
##### START CODING HERE #####
# This code block is optional. You can import other libraries or define your utility functions if necessary.

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
agent = RLAgentWithOtherExploration(all_actions)

print(all_actions)
# start training
for episode in range(1000):
    episode_reward = 0
    s = env.reset()
    # render env. You can comment all render() to turn off the GUI to accelerate training.
    env.render()
    # agent interacts with the environment
    for iter in range(500):
        
        a = agent.choose_action(s, episode)
        s_, r, isdone, info = env.step(a)
        env.render()
        episode_reward += r
        print(f"{s} {a} {s_} {r} {isdone}")
        agent.learn(episode, a, s, episode_reward, s_)
        s = s_
        if isdone:
            #time.sleep(0.5)
            break

    print('episode:', episode, 'episode_reward:', episode_reward, 'epsilon:', agent.epsilon)  
print('\ntraining over\n')   

# close the render window after training.
env.close()

####### START CODING HERE #######
