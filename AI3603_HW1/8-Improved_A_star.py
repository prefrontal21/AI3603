import DR20API
import numpy as np
import sys
import math

### START CODE HERE ###
# This code block is optional. You can define your utility function and class in this block if necessary.
class Point:
    def __init__(self, x,y):
        self.x = x
        self.y = y
        self.parent =None
        self.f = sys.maxsize
        self.h = sys.maxsize
        self.total_cost = sys.maxsize

    def set_parent(self,parent):
        self.parent = parent

    def process_as_start(self,goal_pos):
        self.f = 0
        self.h =abs(self.x - goal_pos[0]) + abs(self.y -goal_pos[1]) +(math.sqrt(2)-2)*min(abs(self.x - goal_pos[0]),abs(self.y -goal_pos[1]))
        self.total_cost = self.h

    #def calculate_angle(self, parent):
    #    return np.arctan((self.y-parent.y)/(self.x-parent.x))

    def process_with_parent(self, goal_pos):
        if abs(self.x - self.parent.x) + abs(self.y - self.parent.y) ==1:
            self.f = self.parent.f +1
        else:
            self.f = self.parent.f + math.sqrt(2)
        self.h = abs(self.x - goal_pos[0]) + abs(self.y -goal_pos[1]) +(math.sqrt(2)-2)*min(abs(self.x - goal_pos[0]),abs(self.y -goal_pos[1]))
        self.total_cost = self.f +self.h
        #self.ang = self.calculate_angle(self.parent)

    def replace_new_parent(self, new_parent,goal_pos):
        #new_ang = self.calculate_angle(parent)
        if self.parent.f < new_parent.f:
            self.parent = new_parent
            self.process_with_parent(goal_pos)

class A_star_class:
    def __init__(self, current_map, current_pos, goal_pos):
        self.current_map = current_map
        self.current_pos = current_pos
        self.goal_pos = goal_pos
        self.open_set = []
        self.closed_set = []

    def add_obstacles_distance(self):
        temp_map = self.current_map.copy()
        for i in range(1,119):
            for j in range(1,119):
                if self.current_map[i][j] + self.current_map[i -1][j] + \
                   self.current_map[i+1][j] + self.current_map[i][j-1] + \
                   self.current_map[i-1][j-1] + self.current_map[i+1][j-1] + \
                   self.current_map[i][j+1] + self.current_map[i-1][j+1] + \
                   self.current_map[i+1][j+1] > 0:
                    temp_map[i][j] =1
        self.current_map = temp_map

    def process_start(self):
        current_point = Point(self.current_pos[0],self.current_pos[1])
        current_point.process_as_start(goal_pos)
        self.open_set.append(current_point)

    def select_min_cost(self):
        min_cost = sys.maxsize
        id = 0
        select_id =-1

        for p in self.open_set:
            if p.total_cost <= min_cost:
                min_cost = p.total_cost
                select_id = id
            id+=1
        return select_id

    def in_set(self, p, set):
        for point in set:
            if (p.x - point.x ==0) & (p.y - point.y ==0):
                return point
        return None

    def in_open_set(self,p):
        return self.in_set(p,self.open_set)

    def in_closed_set(self,p):
        return self.in_set(p,self.closed_set)

    def process(self, x, y, parent):
        if (x<0) | (x>= 120) | (y<0) | (y>=120):
            return
        if self.current_map[x][y]==1:
            return

        p = Point(int(x),int(y))
        if self.in_closed_set(p):
            return
        q  = self.in_open_set(p)
        if q == None:
            p.parent= parent
            p.process_with_parent(self.goal_pos)
            self.open_set.append(p)
        else:
            q.replace_new_parent(parent,self.goal_pos)
            
    def reached(self,p):
        if (p.x == self.goal_pos[0]) & (p.y ==self.goal_pos[1]):
            print("total cos=",p.f)
            return True
        return False

    def delete_redundant_nodes(self, path):
        length = len(path)
        delete_nodes = []
        cnt =0
        current_pos = path[0]
        cut_id =0
        for i in range(length-2):
            if (path[i+1][0] - current_pos[0])**2 + (path[i+1][1] - current_pos[1])**2 >800:
                cut_id = i+2
                break
            if (path[i][0] == path[i+1][0]) & (path[i+2][0] == path[i+1][0]):
                delete_nodes.append(i+1)
            if (path[i][1] == path[i+1][1]) & (path[i+2][1] == path[i+1][1]):
                delete_nodes.append(i+1)
            if ((path[i][0]- path[i][1]) == (path[i+1][0]- path[i+1][1])) & \
                ((path[i+2][0]- path[i+2][1]) == (path[i+1][0]- path[i+1][1])):
                delete_nodes.append(i+1)
        for j in delete_nodes:
            path.pop(j-cnt)
            cnt+=1
           
        return path

    def is_reachable(self, pointA, pointB):
        start_x = pointA[0]
        start_y = pointA[1]
        end_x = pointB[0]
        end_y = pointB[1]
        if end_x == start_x:
            for y in range(0,end_y- start_y):
                if self.current_map[start_x][start_y+ y] ==1:
                    return False
        else:
            k = float(end_y - start_y)/float(end_x- start_x)
            for x in range(0, end_x-start_x):
                y = int(k*x)
                if self.current_map[x+start_x][y+start_y] ==1:
                    return False
            for y in range(0, end_y-start_y):
                x = int(y/k)
                if self.current_map[x+start_x][y+start_y] ==1:
                    return False
        return True        


    def delete_turns(self, path):
        length = len(path)
        delete_nodes = []
        cnt =0
        current_pos = path[0]
        i=0
        while i <= length-3:
            if (path[i+1][0] - current_pos[0])**2 + (path[i+1][1] - current_pos[1])**2 >800:
                break
            if self.is_reachable(path[i], path[i+2]) & (i not in delete_nodes):
                path.pop(i+1)
            else:
                i+=1
            length = len(path)
               # delete_nodes.append(i+1)
                #print("i+1=",i+1)
        #for j in delete_nodes:
          #  path.pop(j-cnt)
         #   cnt+=1
           
        return path


    def build_path(self,p):
        path = []
        path.append([int(p.x),int(p.y)])
        while True:
            p = p.parent
            if p:
                path.insert(0,[int(p.x),int(p.y)])
            if (p.x == self.current_pos[0]) & (p.y == self.current_pos[1]):
                return path
        return path

    def run(self):
        self.process_start()
        self.add_obstacles_distance()
        np.savetxt("map.txt",self.current_map)
        print("current pos:", self.current_pos[0],self.current_pos[1])
        while True:
            id =self.select_min_cost()
            p = self.open_set[id]
            self.closed_set.append(p)
            del self.open_set[id]
            if(self.reached(p)):
                path = self.build_path(p)
                path = self.delete_redundant_nodes(path)
                #print("after delete redundant ",path)
                path = self.delete_turns(path)
                #print("after delete turn ",path)
                path.append([100,101])
                return path

            #print(p.x,p.y,self.current_map[p.x][p.y],p.f,p.h,p.total_cost)
            self.process(p.x+1, p.y-1,p)
            self.process(p.x-1,p.y-1,p)
            self.process(p.x+1, p.y+1,p)
            self.process(p.x-1, p.y+1,p)
            self.process(p.x-1, p.y,p)
            self.process(p.x, p.y-1,p)
            self.process(p.x, p.y+1,p)
            self.process(p.x+1, p.y,p)
###  END CODE HERE  ###

def Improved_A_star(current_map, current_pos, goal_pos):
    """
    Given current map of the world, current position of the robot and the position of the goal, 
    plan a path from current position to the goal using improved A* algorithm.

    Arguments:
    current_map -- A 120*120 array indicating current map, where 0 indicating traversable and 1 indicating obstacles.
    current_pos -- A 2D vector indicating the current position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    path -- A N*2 array representing the planned path by improved A* algorithm.
    """

    ### START CODE HERE ###
    current_pos = [int(current_pos[0]),int(current_pos[1])]
    goal_pos =[int(goal_pos[0]),int(goal_pos[1])]
    algorithm = A_star_class(current_map,current_pos,goal_pos)
    path = algorithm.run()
    #print(algorithm.is_reachable([24,24],[24,31]))
    ###  END CODE HERE  ###
    return path

def reach_goal(current_pos, goal_pos):
    """
    Given current position of the robot, 
    check whether the robot has reached the goal.

    Arguments:
    current_pos -- A 2D vector indicating the current position of the robot.
    goal_pos -- A 2D vector indicating the position of the goal.

    Return:
    is_reached -- A bool variable indicating whether the robot has reached the goal, where True indicating reached.
    """

    ### START CODE HERE ###
    if (current_pos[0] - goal_pos[0] == 0) & (current_pos[1] - goal_pos[1] ==0):
        is_reached =True
    else:
        is_reached = False 
    ###  END CODE HERE  ###
    return is_reached

if __name__ == '__main__':
    # Define goal position of the exploration, shown as the gray block in the scene.
    goal_pos = [100, 100]
    controller = DR20API.Controller()

    # Initialize the position of the robot and the map of the world.
    current_pos = controller.get_robot_pos()
    current_map = controller.update_map()

    #Improved_A_star(current_map, current_pos, goal_pos)

    # Plan-Move-Perceive-Update-Replan loop until the robot reaches the goal.
    while not reach_goal(current_pos, goal_pos):
        # Plan a path based on current map from current position of the robot to the goal.
        path = Improved_A_star(current_map, current_pos, goal_pos)
        # Move the robot along the path to a certain distance.
        controller.move_robot(path)
        # Get current position of the robot.
        current_pos = controller.get_robot_pos()
        # Update the map based on the current information of laser scanner and get the updated map.
        current_map = controller.update_map()
#
    ## Stop the simulation.
    controller.stop_simulation()