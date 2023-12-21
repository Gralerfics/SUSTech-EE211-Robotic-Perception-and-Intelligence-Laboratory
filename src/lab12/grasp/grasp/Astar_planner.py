import math
import time
import numpy as np
from scipy.ndimage import distance_transform_cdt

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped


class AStarPlanner:
    def __init__(self, occ_grid: OccupancyGrid, bot_radius):
        self.resolution = occ_grid.info.resolution
        self.x_width, self.y_width = occ_grid.info.width, occ_grid.info.height
        self.min_x, self.min_y = occ_grid.info.origin.position.x, occ_grid.info.origin.position.y
        self.max_x, self.max_y = self.min_x + self.resolution * self.x_width, self.min_y + self.resolution * self.y_width
        
        # convert map
        map_data = np.array(occ_grid.data).astype(int).reshape(self.y_width, -1).transpose()
        self.obstacle_map = np.ones(map_data.shape, dtype=bool)
        self.obstacle_map[map_data == 0] = False
        
        # bot radius
        distances = distance_transform_cdt(np.logical_not(self.obstacle_map)) * self.resolution
        self.obstacle_map[np.logical_and(self.obstacle_map == False, distances < bot_radius)] = True
        
        # import matplotlib.pyplot as plt
        # plt.imshow(self.obstacle_map, cmap='viridis', interpolation='none')
        # plt.savefig('map_plot.png')
        
        self.motion = self.get_motion_model()
    
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        start_node = self.Node(self.calc_xy_index(sx, self.min_x), self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x), self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if len(open_set) == 0:
                print("Open set is empty..")
                break

            c_id = min(open_set, key = lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0 # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return node.y * self.x_width + node.x

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1], [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)], [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]
        return motion
    
    
    def find_path(self, initial_pose, goal_pose):
        initial_x, initial_y = initial_pose.pose.position.x, initial_pose.pose.position.y
        goal_x, goal_y = goal_pose.pose.position.x, goal_pose.pose.position.y
        rx, ry = self.planning(initial_x, initial_y, goal_x, goal_y)
        path: Path = Path()
        path.header.stamp = time.now().to_msg()
        path.header.frame_id = 'map'
        n = len(rx)
        for i in range(20, n):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = rx[i], ry[i]
            if i == n - 1:
                pose.pose.orientation = goal_pose.pose.orientation
            else:
                pass
                # pose.pose.orientation = Qua...
            path.poses.append(pose)
        return path

