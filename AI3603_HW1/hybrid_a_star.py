

# It finds the optimal path for a car using Hybrid A* and bicycle model.
import heapq as hq
import math
import dubins
import point3D as p3

steering_inputs = [-40, 0, 40]
cost_steering_inputs = [0.1, 0, 0.1]

speed_inputs = [-1, 1]
cost_speed_inputs = [1, 0]

# Accordint to the paper, total cost f(n) = heuristic cost h(n) + actual cost g(n)
# heuristic cost h(n) =
#    max(
#       max(non-holonomic-without-obstacles(n, end), euclid distance),
#       A_star distance)
#    )


class hybrid_a_star:
    def __init__(self, min_x, max_x, min_y, max_y, current_map=[], resolution=1, vehicle_length=2):
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.points_map = {}  # Used to store points context
        self.open_heap = []   # element of this list is like (cost,node_d)
        self.closed_set = {}  # Used to store the traversal of points
        self.current_map = current_map
        self.resolution = resolution
        self.vehicle_length = vehicle_length

    def dubin_dist(self, start, target):
        err, path = dubins.dubins_init([start[0], start[1], math.radians(start[2])],
                                       [target[0], target[1], math.radians(target[2])], 1)
        dubins_length = sum(path.params)

        return dubins_length

    def terminate(self, chosen_d_node, start, end, flag):
        rev_final_path = []
        final_path = []

        # If flag =1, it indicates that the end point has been reached, and
        # flag=0 indicates that the longest distance of the car has been reached
        if flag == 1:
            rev_final_path = [end]

        node = chosen_d_node
        while True:
            open_node_contents = self.points_map[node]
            parent_c = open_node_contents.parent_c
            node = open_node_contents.parent_d

            rev_final_path.append(parent_c)
            if node == start:
                rev_final_path.append(start)
                break

        for i in range(len(rev_final_path)-1):
            final_path.insert(0, [rev_final_path[i][0], rev_final_path[i][1]])

        return final_path, rev_final_path[0][2]

    def calculate_neighbour_cost(self, i, j, neighbour_d, current_cost, parent, end):
        heurestic = self.dubin_dist(neighbour_d, end)
        cost_to_parent_from_start = current_cost - self.dubin_dist(parent, end)
        cost_to_neighbour_from_start = cost_to_parent_from_start + abs(speed_inputs[j]) + \
            cost_steering_inputs[i] + cost_speed_inputs[j]
        total_cost = heurestic + cost_to_neighbour_from_start

        return total_cost

    def process(self, i, j, parent, end, chosen_node_total_cost):
        neighbour_d, neighbour_c = self.points_map[parent].create_successor(
            i, j, self.vehicle_length)

        chosen_c_node = self.points_map[parent].node

        if ((neighbour_d[0] >= self.min_x) and (neighbour_d[0] < self.max_x) and
                (neighbour_d[1] >= self.min_y) and (neighbour_d[1] < self.max_y)) and \
                (self.current_map[int(neighbour_d[0])][int(neighbour_d[1])] == 0):

            total_cost = self.calculate_neighbour_cost(i, j, neighbour_d,
                                                       chosen_node_total_cost, parent, end)

            if neighbour_d in self.points_map:
                if total_cost > self.points_map[neighbour_d].total_cost:
                    return
                elif neighbour_d in self.closed_set and total_cost > self.closed_set[neighbour_d].total_cost:
                    return

            hq.heappush(self.open_heap, (total_cost, neighbour_d))
            self.points_map[neighbour_d] = p3.point3D(
                total_cost, neighbour_c, parent, chosen_c_node)

    def find_path(self, start, end):
        start = (float(start[0]), float(start[1]), float(start[2]))
        end = (float(end[0]), float(end[1]), float(end[2]))

        hq.heappush(self.open_heap, (self.dubin_dist(start, end), start))
        self.points_map[start] = p3.point3D(
            self.dubin_dist(start, end), start, start, start)

        while len(self.open_heap) > 0:
            chosen_d_node, chosen_node_total_cost = self.open_heap[0][1], self.open_heap[0][0]

            self.closed_set[chosen_d_node] = self.points_map[chosen_d_node]

            if self.dubin_dist(chosen_d_node, end) < 1:
                return self.terminate(chosen_d_node, start, end, 1)
            elif self.dubin_dist(chosen_d_node, start) > 20:
                return self.terminate(chosen_d_node, start, end, 0)

            hq.heappop(self.open_heap)

            for i in range(0, 3):
                for j in range(0, 2):
                    self.process(i, j, chosen_d_node, end,
                                 chosen_node_total_cost)

        print("Did not find the goal - it's unattainable.")
        return []
