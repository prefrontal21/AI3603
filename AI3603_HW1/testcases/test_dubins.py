import matplotlib.pyplot as plt
import sys
import os
import math
import numpy as np
sys.path.append(os.path.realpath('../AI3603_HW1'))
import dubins


def test_dubins_LSR():
    err, path = dubins.dubins_init([0, 0, math.pi/2], [-3, 2, math.pi/2], 1)
    points = dubins.dubins_path_sample_many(path)
    x = [points[i][0] for i in range(len(points))]
    y = [points[i][1] for i in range(len(points))]
    plt.plot(0, 0, "xr")
    plt.plot(-3, 2, "xb")
    
    plt.plot(x, y, "-r")
    plt.savefig("out/test_LSR.png")
    plt.clf()

def test_dubins_LSL():
    err, path = dubins.dubins_init([0, 0, math.pi/2], [-3, 0, -math.pi/2], 1)
    points = dubins.dubins_path_sample_many(path)
    x = [points[i][0] for i in range(len(points))]
    y = [points[i][1] for i in range(len(points))]
    plt.plot(0, 0, "xr")
    plt.plot(-3, 0, "xb")
    
    plt.plot(x, y, "-r")
    plt.savefig("out/test_LSL.png")
    plt.clf()

def test_dubins_RSL():
    err, path = dubins.dubins_init([0, 0, math.pi/2], [3, 2, math.pi/2], 1)
    points = dubins.dubins_path_sample_many(path)
    x = [points[i][0] for i in range(len(points))]
    y = [points[i][1] for i in range(len(points))]
    plt.plot(0, 0, "xr")
    plt.plot(3, 2, "xb")
    
    plt.plot(x, y, "-r")
    plt.savefig("out/test_RSL.png")
    plt.clf()

def test_dubins_RSR():
    err, path = dubins.dubins_init([0, 0, math.pi/2], [3, 0, -math.pi/2], 1)
    points = dubins.dubins_path_sample_many(path)
    x = [points[i][0] for i in range(len(points))]
    y = [points[i][1] for i in range(len(points))]
    plt.plot(0, 0, "xr")
    plt.plot(3, 0, "xb")
    
    plt.plot(x, y, "-r")
    plt.savefig("out/test_RSR.png")
    plt.clf()

if __name__ == '__main__':
    test_dubins_LSR()
    test_dubins_RSL()
    test_dubins_RSR()
