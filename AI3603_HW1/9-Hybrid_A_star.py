import DR20API
import matplotlib.pyplot as plt
import hybrid_a_star

global general_path
global final_theta
general_path = []
final_theta = 90


def add_obstacles_distance(current_map):
    temp_map = current_map.copy()
    for i in range(1, 119):
        for j in range(1, 119):
            if current_map[i][j] + current_map[i - 1][j] + \
               current_map[i+1][j] + current_map[i][j-1] + \
               current_map[i-1][j-1] + current_map[i+1][j-1] + \
               current_map[i][j+1] + current_map[i-1][j+1] + \
               current_map[i+1][j+1] > 0:
                temp_map[i][j] = 1
    return temp_map

### START CODE HERE ###


def Hybrid_A_star(current_map, current_pos, goal_pos):
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
    global final_theta

    sx, sy, stheta = int(current_pos[0]), int(current_pos[1]), final_theta
    gx, gy, gtheta = int(goal_pos[0]), int(goal_pos[1]), 0

    hy_a_star = hybrid_a_star.hybrid_a_star(0, 120, 0, 120, current_map=current_map,
                                            resolution=1, vehicle_length=2)
    path, final_theta = hy_a_star.find_path((sx, sy, stheta), (gx, gy, gtheta))

    '''
    global general_path
    ox, oy = [], []

    for i in range(120):
        for j in range(120):
            if current_map[i][j]:
                ox.append(i)
                oy.append(j)

    current_map = add_obstacles_distance(current_map)
    
    plt.plot(ox, oy, ".k")
    plt.plot(sx, sy, "xr")
    plt.plot(gx, gy, "xb")
    plt.grid(True)
    plt.axis("equal")
    general_path.extend(path)
    rx, ry = [], []
    for node in general_path:
        rx.append(node[0])
        ry.append(node[1])

    plt.plot(rx, ry, "-r")
    plt.show()
    '''
    return path
    ###  END CODE HERE  ###


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
    if (abs(current_pos[0] - goal_pos[0]) < 5) and (abs(current_pos[1] - goal_pos[1]) < 5):
        is_reached = True
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

    # Hybrid_A_star(current_map, current_pos, goal_pos)

    # Plan-Move-Perceive-Update-Replan loop until the robot reaches the goal.
    while not reach_goal(current_pos, goal_pos):
        # Plan a path based on current map from current position of the robot to the goal.
        path = Hybrid_A_star(current_map, current_pos, goal_pos)
        # Move the robot along the path to a certain distance.
        controller.move_robot(path)
        # Get current position of the robot.
        current_pos = controller.get_robot_pos()
        # Update the map based on the current information of laser scanner and get the updated map.
        current_map = controller.update_map()

    # Stop the simulation.
    controller.stop_simulation()

###  END CODE HERE  ###
