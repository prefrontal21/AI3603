import math
import matplotlib.pyplot as plt

steering_inputs = [-40, 0, 40]
cost_steering_inputs = [0.1, 0, 0.1]

speed_inputs = [-1, 1]


class point3D:
    def __init__(self, total_cost, node, parent_d, parent_c):
        self.total_cost = total_cost
        self.parent_d = parent_d
        self.parent_c = parent_c
        self.node = node

    def __lt__(self, other):
        return self.f < other.f

    def create_successor(self, i, j, vehicle_length):
        delta = steering_inputs[i]
        velocity = speed_inputs[j]

        successor_x_cts = self.node[0] + \
            (velocity * math.cos(math.radians(self.node[2])))
        successor_y_cts = self.node[1] + \
            (velocity * math.sin(math.radians(self.node[2])))
        successor_theta_cts = math.radians(
            self.node[2]) + (velocity * math.tan(math.radians(delta))/(float(vehicle_length)))
        successor_theta_cts = math.degrees(successor_theta_cts)

        successor_x_d = round(successor_x_cts)
        successor_y_d = round(successor_y_cts)
        successor_theta_d = round(successor_theta_cts)

        successor_d = (successor_x_d, successor_y_d, successor_theta_d)
        successor_c = (successor_x_cts, successor_y_cts, successor_theta_cts)

        return successor_d, successor_c
