import math
import matplotlib.pyplot as plt
import numpy as np
import sys

class dubins_path:
    def __init__(self, params, type):
        self.params = params
        self.type = type
        self.qi = None
        self.rho = 0
    
    def set_start_and_rho(self, q0, rho):
        self.qi = q0
        self.rho = rho

    def dubins_path_length(self):
        return sum(self.params)

'''
This is a brief implementation of dubins curve.
We define:
    LSL = 0
    LSR = 1
    RSL = 2
    RSR = 3
    RLR = 4
    LRL = 5
'''

segment_type_map = [[0,2,0],[0,2,1],[1,2,0],[1,2,1],[1,0,1],[0,1,0]]

def fmodr(x, y):
    return x - y* math.floor(x/y)

def mod2pi(theta):
    return fmodr(theta, 2* math.pi)

def dubins_LSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha- beta)
    
    res = []
    tmp0 = d + sa - sb
    p_squared = 2 + d**2 - (2* c_ab) + (2*d*(sa - sb))
    if p_squared < 0.01:
        return 0,res
    tmp1 = np.arctan2((cb - ca), tmp0)
    res.append(mod2pi(alpha-tmp1))
    res.append(math.sqrt(abs(p_squared)))
    res.append(mod2pi(-beta + tmp1))
    return 1,res

def dubins_LSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha- beta)
    
    tmp0 = d + sa +sb
    p_squared = -2 + d**2 + (2* c_ab) + (2*d*(sa + sb))
    if p_squared <-0.01:
        return 0, []
    p = math.sqrt(abs(p_squared))
    tmp2 = np.arctan2((-ca - cb), tmp0) - np.arctan2(-2.0, p)
    res = []
    res.append(mod2pi(-alpha+ tmp2))
    res.append(p)
    res.append(mod2pi(-beta + tmp2))
    return 1, res

def dubins_RSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha- beta)
    
    tmp0 = d - sa -sb
    p_squared = -2 + d**2 + (2* c_ab) - (2*d*(sa + sb))
    if p_squared < -0.01:
        return 0, []
    p = math.sqrt(abs(p_squared))
    tmp2 = np.arctan2((ca + cb), tmp0) - np.arctan2(2.0, p)
    res = []
    res.append(mod2pi(alpha-tmp2))
    res.append(p)
    res.append(mod2pi(beta - tmp2))
    return 1, res

def dubins_RSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha- beta)
    
    tmp0 = d - sa + sb
    p_squared = 2 +d**2 - (2* c_ab) + (2*d*(sb - sa))
    if p_squared <-0.1:
        return 0,[]
    tmp1 = np.arctan2((ca - cb), tmp0)
    res = []
    res.append(mod2pi(alpha-tmp1))
    res.append(math.sqrt(abs(p_squared)))
    res.append(mod2pi(-beta + tmp1))
    return 1,res 

def dubins_RLR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha- beta)
    
    res = []
    tmp_rlr = (6- d*d + 2*c_ab + 2*d*(sa -sb))/float(8)
    if math.fabs(tmp_rlr) >1:
        return 0, []
    p = mod2pi(2*math.pi - np.arccos(tmp_rlr))
    t = mod2pi(alpha - np.arctan2(ca-cb, d-sa+sb) + math.pi/2.)
    res.append(p)
    res.append(t)
    res.append(mod2pi(alpha - beta -t + mod2pi(p)))
    return 1, res
    
def dubins_LRL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha- beta)
    
    res = []
    tmp_rlr = (6- d*d + 2*c_ab + 2*d*(-sa +sb))/float(8)
    if math.fabs(tmp_rlr) >1:
        return 0, []
    p = mod2pi(2*math.pi - np.arccos(tmp_rlr))
    t = mod2pi(-alpha - np.arctan2(ca-cb, d+sa-sb) + math.pi/2.)
    res.append(p)
    res.append(t)
    res.append(mod2pi(mod2pi(beta) -alpha -t + mod2pi(p)))
    return 1, res

dubins_func_map = [dubins_LSL, dubins_LSR, dubins_RSL, dubins_RSR, dubins_RLR, dubins_LRL]

def dubins_init_normalise(alpha, beta, d):
    best_cost = sys.maxsize
    best_word = -1
    err =-1
    res = []
    path = dubins_path([],-1)

    for i in range(6):
        #print(i)
        #print(dubins_func_map[i](alpha, beta, d))
        err, res = dubins_func_map[i](alpha, beta, d)
        if err == 1:
            cost = sum(res)
            #print(res)
            if cost < best_cost:
                best_word = i
                best_cost = cost
                path.params = res
                path.type =i

    if best_word == -1:
        return 0, path
    return 1, path

def dubins_init(q0, q1, rho):
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    D = math.sqrt(dx**2+ dy**2)
    path = dubins_path([],-1)
    d = D/rho

    if rho <= 0:
        return 0, path
    theta = mod2pi(np.arctan2(dy, dx))
    alpha = mod2pi(q0[2] - theta)
    beta = mod2pi(q1[2] - theta)
    err, path = dubins_init_normalise(alpha, beta, d)
    #print(path.params)
    #print(path.type)
    path.set_start_and_rho(q0, rho)

    return err, path

def dubins_segment(length, qi, type):
    qt = [0 for x in range(3)]

    if type == 0: # L_segment
        qt[0] = qi[0] + math.sin(float(qi[2])+length) - math.sin(qi[2])
        qt[1] = qi[1] - math.cos(float(qi[2])+length) + math.cos(qi[2])
        qt[2] = qi[2] + length
        return 1, qt
    elif type == 1: # R_segment
        qt[0] = qi[0] - math.sin(float(qi[2])-length) + math.sin(qi[2])
        qt[1] = qi[1] + math.cos(float(qi[2])-length) - math.cos(qi[2])
        qt[2] = qi[2] - length
        return 1, qt
    elif type == 2: # S_segment
        qt[0] = qi[0] + math.cos(qi[2])*length
        qt[1] = qi[1] + math.sin(qi[2])*length
        qt[2] = qi[2]
        return 1, qt
    else:
        print("error! dubin segment type must be L,R,S")
        return 0, []

def dubins_path_sample(path, t):
    if (t<= 0) | (t > path.dubins_path_length()):
        return 0,[0,0,0]

    tprime = t/float(path.rho)
    types = segment_type_map[path.type]
    qi = [0, 0, path.qi[2]]
    q1 = [0 for x in range(3)]
    q2 = [0 for x in range(3)]
    p1 = path.params[0]
    p2 = path.params[1]
    q = [0 for x in range(3)]
    err1, q1 = dubins_segment(p1, qi, types[0])
    err2, q2 = dubins_segment(p2, q1, types[1])
    if tprime < p1: 
        err3, q = dubins_segment(tprime, qi, types[0])
    elif tprime < p1 + p2:
        err4, q = dubins_segment(tprime -p1, q1, types[1])
    else:
        err5, q = dubins_segment(tprime -p1-p2, q2, types[2])
    
    q[0] = q[0]* path.rho + path.qi[0]
    q[1] = q[1]* path.rho + path.qi[1]
    q[2] = mod2pi(q[2])
    
    return 1,q

def dubins_path_sample_many(path):
    x = 0.0
    points = []
    length = path.dubins_path_length()

    while x <length:
        err, q =dubins_path_sample(path, x)
        points.append(q)
        x+=0.1

    err, q = dubins_path_sample(path, length)
    points.append(q)
    #print(points)
    return points

# print(dubins_LSL(0,0,10))
# 
# print(dubins_RSR(0,0,10))
# 
# print(dubins_LSR(0,0,10))
# 
# print(dubins_RSL(0,0,10))
# 
# print(dubins_RLR(0,0,0))
# 
# print(dubins_LRL(0,0,0))
# 
# dubins_init_normalise(0,0,10)
# dubins_init([1,0,math.pi/2],[1,10,math.pi/2],1)

#path = dubins_path([0,10,0],1)
#path.set_start_and_rho([0,0,0],1)
#q = [0 for x in range(3)]
#dubins_path_sample(path, 5, q)

#err, path = dubins_init([0,0, math.pi/2],[3,2,math.pi/2],1) 
#print(path.params, path.type)
#points =dubins_path_sample_many(path)
#x = [points[i][0] for i in range(len(points))]
#y = [points[i][1] for i in range(len(points))]
#start_and_goal_x = [0,3]
#start_and_goal_y = [0,2]
#plt.plot(start_and_goal_x,start_and_goal_y,"bo")
#plt.plot(x,y,"-r")
#plt.savefig("RSL.png")
#plt.show()