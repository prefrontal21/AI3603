# -*- coding:utf-8 -*-
# gym录像机教程，仅供参考，不需提交该文件。

import math, os, time, sys
import numpy as np
import random
import gym
from gym.wrappers import Monitor
from gym_gridworld import CliffWalk
import gym_sokoban
from agent import *

"""
1. This is an example to record video with gym library.
2. You can choose other video record tools as you wish. 
3. You DON'T NEED to upload this file in your assignment.
"""

# record sokoban environment
# the video file will be saved at the ./video folder.
env = Monitor(gym.make('Sokoban-hw2-v0'), './video', force=True)

num_actions = env.action_space.n
all_actions = np.arange(num_actions)
RANDOM_SEED = 0
env.seed(RANDOM_SEED)
random.seed(RANDOM_SEED) 
np.random.seed(RANDOM_SEED) 

agent = QLearningAgent(all_actions)

s = env.reset()
env.render()
for iter in range(200):
    a = agent.choose_action(s)
    s_, r, isdone, info = env.step(a)
    env.render()
    agent.learn()
    s = s_
    if isdone:
        break

# the recorder will stop when calling `env.close()` function
# the video file .mp4 will be saved at the ./video folder
env.close()

