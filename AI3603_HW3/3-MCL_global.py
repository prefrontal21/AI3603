import DR20API
import numpy as np
import matplotlib.pyplot as plt
import math
from math import sin, cos, pi
import time

###  START CODE HERE  ###
# You can tune the hyper-parameter, and define your utility function or class in this block if necessary. 

obstacles_x = {0: [[0, 5]], 0.5: [],  1: [[2, 2.5]],
               1.5: [[2, 2.5]], 2: [[1, 1.5], [2, 3.5]],
               2.5: [[1, 1.5]], 3: [[1, 1.5], [2, 3.5]],
               3.5: [[1, 1.5]], 4: [[4, 4.5]],
               4.5: [[4, 4.5]], 5: [[0, 5]]}

obstacles_y = {0: [[0, 5]], 0.5: [], 1: [[2, 2.5], [3, 3.5]],
               1.5: [[2, 2.5], [3, 3.5]], 2: [[1, 1.5], [2, 3.5]],
               2.5: [[1, 1.5]], 3: [],
               3.5: [[2, 3]], 4: [[4, 4.5]],
               4.5: [[4, 4.5]], 5: [[0, 5]]}


def mod2pi(theta):
    return math.radians(math.degrees(theta) % 360)


def is_right_direction(theta):
    return (0 <= theta and theta <= math.pi/2) or (theta >= 3*math.pi/2 and theta <= 2 * math.pi)


def is_up_direction(theta):
    return 0 <= theta and theta <= math.pi


def get_x_nearest_obstacle(x, theta):
    if is_right_direction(theta):
        return int(x*2+1)/2.0
    else:
        return int(x*2)/2.0


def get_y_nearest_obstacle(y, theta):
    if is_up_direction(theta):
        return int(y*2+1)/2.0
    else:
        return int(y*2)/2.0


def find_cloest_x_obstacle(x, y, theta):
    k = math.tan(theta)
    pos_x = get_x_nearest_obstacle(x, theta)
    while pos_x <= 5.0 and pos_x >= 0:
        pos_y = y + k * (pos_x - x)
        for range_y in obstacles_x[pos_x]:
            if pos_y >= range_y[0] and pos_y <= range_y[1]:
                return pos_x, pos_y
        pos_x += 0.5 if is_right_direction(theta) else -0.5
    return 100, 100


def find_cloest_y_obstacle(x, y, theta):
    k = math.tan(math.pi/2 - theta)
    pos_y = get_y_nearest_obstacle(y, theta)
    while pos_y <= 5.0 and pos_y >= 0:
        pos_x = x + k * (pos_y - y)
        for range_x in obstacles_y[pos_y]:
            if pos_x >= range_x[0] and pos_x <= range_x[1]:
                return pos_x, pos_y
        pos_y += 0.5 if is_up_direction(theta) else -0.5

    return 100, 100


def find_cloest_obstacle(x, y, theta):
    obstacle_1_x, obstacle_1_y = find_cloest_x_obstacle(x, y, theta)
    distance1 = math.sqrt((obstacle_1_x - x)**2 + (obstacle_1_y - y)**2)
    obstacle_2_x, obstacle_2_y = find_cloest_y_obstacle(x, y, theta)
    distance2 = math.sqrt((obstacle_2_x - x)**2 + (obstacle_2_y - y)**2)
    return distance1 if distance1 < distance2 else distance2


def detect_obstacles(x, y, theta):
    distance = np.array([find_cloest_obstacle(
        x, y, mod2pi(theta + i * math.pi/4)) for i in range(-2, 3)])

    return distance


def randomize_input(u):
    v = u[0]
    w = u[1]
    v = v + np.random.randn() * R[0, 0] ** 0.5
    w = w + np.random.randn() * R[1, 1] ** 0.5
    ud = np.array([v, w]).reshape(2, 1)
    return ud

# Particle filter parameter
NP = 1000  # Number of Particle
NTh = NP / 2.0  # Number of particle for re-sampling

# Set the random seed to ensure the repeatability.
seed=1
np.random.seed(seed)

#  Estimation parameter of PF, you may use them in the PF algorithm. You can use the recommended values as follows.
Q = np.diag([0.15]) ** 2  # range error
R = np.diag([0.1, np.deg2rad(10)]) ** 2  # input error

###  END CODE HERE  ###

#  Parameter of LiDAR
scanningAngle = 180
pts=5

#  Simulation parameter
Q_sim = np.diag([0.05]) ** 2  # add noise to lidar readings
R_sim = np.diag([0.01, np.deg2rad(1)]) ** 2  # add noise to control command

v=0.5  #linear velocity
w=0.25 #angular velocity

DT = 0.1  # time tick [s]
SIM_TIME = 200.0  # simulation time [s]



class Room(object):
    """
    Generate the map.
    """
    
    def map_range_x(self, start, stop, number, y):
        return [[start + (stop - start) * i / number, y] for i in range(number + 1)]

    def map_range_y(self, start, stop, number, x):
        return [[x, start + (stop - start) * i / number] for i in range(number + 1)]

    def map_square(self, top_left, bottom_right, points):
        tl_x, tl_y = top_left
        br_x, br_y = bottom_right
        res  = self.map_range_y(tl_y, br_y, points, tl_x)
        res += self.map_range_y(tl_y, br_y, points, br_x)
        res += self.map_range_x(tl_x, br_x, points, tl_y)
        res += self.map_range_x(tl_x, br_x, points, br_y)
        return res

    def make_room(self):
        walls = self.map_square((0.0,0.0), (5.0,5.0), 30)
        table1 = self.map_square((2.0,2.0), (3,3.5), 10)
        table2 = self.map_square((2.0,1.0), (2.5,1.5), 5)
        table3 = self.map_square((3.0,1.0), (3.5,1.5), 5)
        table4 = self.map_square((1.0,2.0), (1.5,2.5), 5)
        table5 = self.map_square((4.0,4.0), (4.5,4.5), 5)
        return walls + table1 + table2 + table3 + table4 + table5


def motion_model(x, u):
    """
    Given the current state and control input, return the state at next time.

    Arguments:
    x -- A 3*1 matrix indicating the state of robots or particles. Data format: [[x] [y] [yaw]]
    u -- A 2*1 matrix indicating the control input. Data format: [[v] [w]] 
    
    Return:
    x -- A 3*1 matrix indicating the state at next time. Data format: [[x] [y] [yaw]]
    """

    F = np.array([[1.0, 0, 0],
                  [0, 1.0, 0],
                  [0, 0, 1.0]])
          
    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],])
    x = F.dot(x) + B.dot(u)
    return x

def calc_input(v,w):
    """
    Adding noise to the input control commands.

    Argument:
    v -- linear velocity
    w -- angular velocity
    
    Return:
    ud -- A 2*1 matrix indicating the noisy input.
    """

    v = v + np.random.randn() * R_sim[0, 0] ** 0.5
    w = w + np.random.randn() * R_sim[1, 1] ** 0.5
    ud = np.array([[v, w]]).T
    return ud

def generate_particles(pos):
    """
    Generate NP particles, each particle contains three state quantities and weight.  
    If you set the Nearby_flag as True, then the particles will be sprinkled around the real robot.
    Otherwise, the particles will be scattered uniformly in the map.
    In this file, Nearby_flag is set to False to solve a global localization problem. 

    Arguments:
    pos -- The robot current position.

    Return:
    px -- A 3*NP matrix, each column represents the status of a particle.
    pw -- A 1*NP matrix, each column represents the weight value of correspoding particle.
    """
    Nearby_flag = False

    if Nearby_flag:
        x = pos[0] - 1 + 2 * np.random.random(size=(1,NP))
        y = pos[1] - 1 + 2 * np.random.random(size=(1,NP))
        yaw = 2 * math.pi * np.random.random(size=(1,NP))
    else:
        x = 5 * np.random.random(size=(1,NP))
        y = 5 * np.random.random(size=(1,NP))
        yaw = 2 * math.pi * np.random.random(size=(1,NP))
    
    px = np.zeros((3, NP))  # Particle store
    pw = np.zeros((1, NP)) + 1.0 / NP  # Particle weight
    px[0],px[1],px[2],=x,y,yaw
    return px,pw

def pf_localization(px,pw,data,u):
    """
    Localization with Particle filter. In this function, you need to:

    (1) Prediction step: Each particle predicts its new location based on the actuation command given.
    (2) Update step:     Update the weight of each particle. That is, particles consistent with sensor readings will have higher weight.
    (3) Resample step:   Generate a set of new particles, with most of them generated around the previous particles with more weight. 
                         You need to decide when to resample the particles and how to resample the particles.
    
    Argument:
    px -- A 3*NP matrix, each column represents the status of a particle.
    pw -- A 1*NP matrix, each column represents the weight value of correspoding particle.
    data -- A List contains the output of the LiDAR sensor. It is represented by a distance value in counterclockwise order.
    u -- A 2*1 matrix indicating the control input. Data format: [[v] [w]] 

    Return:
    x_est -- A 3*1 matrix, indicating the estimated state after particle filtering.
    px -- A 3*NP matrix. The predicted state of the next time.
    pw -- A 1*NP matrix. The updated weight of each particle.
    """
    t1 = time.time()
    ### START CODE HERE ###

    for ip in range(NP):
        #  Prediction step: Predict with random input sampling
        ud = randomize_input(u)
        par = px[:, ip].reshape(3, 1)
        pred = motion_model(par, ud)
        px[:, ip] = pred.reshape(3)

        pw[0][ip] = 1 / \
            np.linalg.norm(detect_obstacles(
                px[0][ip], px[1][ip], px[2][ip]) - data)
        #  Update steps: Calculate importance weight
        pass
    
    pw = pw / pw.sum()  # normalize
    x_est = px.dot(pw.T)

    # Resample step: Resample the particles.
    px, pw = re_sampling(px, pw)

    ###  END CODE HERE  ###
    print("The time used for each iteration:",time.time()-t1," s")
    return x_est,px, pw
    
def re_sampling(px, pw):
    """
    Robot generates a set of new particles, with most of them generated around the previous particles with more weight.

    Argument:
    px -- The state of all particles befor resampling.
    pw -- The weight of all particles befor resampling.

    Return:
    px -- The state of all particles after resampling.
    pw -- The weight of all particles after resampling.
    """
    ### START CODE HERE ###
    indices = np.array([i for i in range(NP)])
    pw_indices = np.random.choice(indices, NP, replace=True, p=pw[0])
    proposol_particles = np.array(
        [[px[j][pw_indices[i]] + (np.random.rand()-0.5)/5 for i in range(NP)] for j in range(3)])

    ###  END CODE HERE  ###
    return proposol_particles, pw

if __name__ == '__main__':
    plt.figure(figsize=(8,8)) 
    # Build the room map points.
    r = Room()
    room = r.make_room()  
    # Initialize the controller of robot DR20.
    controller = DR20API.Controller()
    data = controller.get_lidar(Q_sim)
    pos = controller.get_robot_pos()
    ori = controller.get_robot_ori()
    # Generate particles, and the data format is [x y yaw]',[weight].
    px, pw = generate_particles(pos)
    # Use the weighted average to calculate the estimated state.(You can use other way, such as clustering.)
    x_est = np.reshape(px.dot(pw.T),(3,1))
    # Initialize the data history.
    h_x_est = x_est
    h_x_true=np.array([[pos[0]],[pos[1]],[ori]])
    tic = 0.0
    # Start simulation.
    while SIM_TIME >= tic:
        tic += DT
        u = calc_input(v,w)
        # Move the robot
        controller.move_robot_vw(u[0, 0],u[1, 0])
        # Locate the robot
        x_est, px, pw = pf_localization(px, pw, data, u)

        # store data history.
        h_x_est = np.hstack((h_x_est, x_est))
        h_x_true = np.hstack((h_x_true, np.array([[pos[0]],[pos[1]],[ori]])))

         # Get the LiDAR sensor data and real state.
        data = controller.get_lidar(Q_sim)
        pos = controller.get_robot_pos()
        ori = controller.get_robot_ori()
        ###  START CODE HERE  ###
        # Visualization
        if True:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [controller.stop_simulation() if event.key == 'escape' else None])
            x, y = zip(*room)
            plt.scatter(x, y)
            plt.plot(np.array(h_x_true[0, :]).flatten(),
                     np.array(h_x_true[1, :]).flatten(), "-b")
            
            # Plot the particles
            plt.scatter(px[0,:],px[1,:],color = 'g',s=5)

            plt.axis("equal")
            plt.grid(True)
            plt.xlim(-0.5,5.5)
            plt.ylim(-0.5,5.5)
            plt.pause(0.01)
        ###  END CODE HERE  ###
    controller.stop_simulation()    