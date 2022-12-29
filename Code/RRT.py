# -----------------------------------------------------------------------------
# Import needed packages
# -----------------------------------------------------------------------------

import pygame
import numpy as np
import Steering as steer
from tqdm import tqdm, trange
from matplotlib import pyplot as plt
from matplotlib import patches

# -----------------------------------------------------------------------------
# Define class that executes the RRT algorithm
# -----------------------------------------------------------------------------
'''
Generate a pose at position (x, y) and at an angle t in degrees
'''
def pose_deg(x, y, t):
    return np.array([x, y, np.deg2rad(t)])


'''
A single node in the RR tree.
A node has a given position/angle.
Each node also has a backreference to the edge leading to it, as well as the previous node visited.
Finally, each node has a set distance from the origin point.
'''
class Node():
    def __init__(self, pose : np.ndarray, distance_from_parent : float = 0.0, parent_node = None, parent_edge=None):
        self.pose = pose
        self.parent_node = parent_node
        self.parent_edge = parent_edge
        self.distance_from_origin = distance_from_parent
        if parent_node is not None:
            # print(f"{parent_node=}")
            self.distance_from_origin += parent_node.distance_from_origin


'''
An edge is a connection between two nodes of the RR tree.
Each edge has a reference to a start node and an end node. Note that the end node will also have a reference to the start node and this edge.
Finally, the edge has a path object, which in this case is a Dubbins Path
'''
class Edge():
    def __init__(self, start_node : Node, path : steer.Path):
        self.start_node = start_node
        self.path = path
        self.end_node = Node(path.end_pose, self.path.length, self.start_node, self)


'''
The environment map that is being planned for is a collection of obstacles and also contains parameters that do not change over the lifetime of the map.
The map can be used to check for collisions, as well as generating random positions or poses.
'''
class Map():
    def __init__(self, obstacles : np.ndarray, vehicle_radius : float, workspace_center, workspace_size):
        self.obstacles = obstacles
        self.vehicle_radius = vehicle_radius
        self.workspace_center = workspace_center
        self.workspace_size = workspace_size

    def collision_check(self, points : np.ndarray) -> bool:
        points = np.atleast_2d(points)
        for point in points:
            for obstacle in self.obstacles:
                if point[0] + self.vehicle_radius > obstacle[0] - obstacle[3]/2 and point[0] - self.vehicle_radius < obstacle[0] + obstacle[3]/2\
                    and point[1] + self.vehicle_radius > obstacle[1] - obstacle[4]/2 and point[1] - self.vehicle_radius < obstacle[1] + obstacle[4]/2:
                    return True
        return False

    def collision_check_single(self, point : np.ndarray) -> bool:
        for obstacle in self.obstacles:
            if point[0] + self.vehicle_radius > obstacle[0] - obstacle[3]/2 and point[0] - self.vehicle_radius < obstacle[0] + obstacle[3]/2\
                and point[1] + self.vehicle_radius > obstacle[1] - obstacle[4]/2 and point[1] - self.vehicle_radius < obstacle[1] + obstacle[4]/2:
                return True
        return False

    def random_position(self) -> np.ndarray:
        collision = True
        while collision:
            new_xy = np.random.uniform(low=self.workspace_center-self.workspace_size/2, high=self.workspace_center+self.workspace_size/2, size = (1,2))[0]
            collision = self.collision_check(new_xy)
        return new_xy

    def random_pose(self) -> np.ndarray:
        new_theta = (np.random.beta(2, 2) *2*np.pi - np.pi) % (2*np.pi)
        return np.hstack((self.random_position(), new_theta))

    def plot(self, ax : plt.Axes):
        for obstacle in self.obstacles:
            ax.add_patch(patches.Rectangle((obstacle[0]-obstacle[3]/2,obstacle[1]-obstacle[4]/2), obstacle[3], obstacle[4]))


'''
This class contains the RR tree.
The tree has a base node, which the tree is grown from.
The tree also has a list of edge objects, which can be used to backtrack through the tree.
The tree has a list of node poses (not node objects), which can be used to rapidly find the nearest node.
The tree also contains a collision map and parameters of the search
'''
class Tree():
    def __init__(self, map : Map, turning_radius : float, initial_pose : np.ndarray, collision_resolution : float):
        self.map = map
        self.base_node = Node(initial_pose)
        self.edges : list[Edge] = []
        self.node_poses = np.array([initial_pose])
        self.node_distances = np.array([0])
        self.turning_radius = turning_radius
        self.collision_resolution = collision_resolution


    '''
    Function to add a new node to the tree. Note that this function breaks the dubbins path up into segments, thus generating more nodes in the tree.
    '''
    def add_node(self, start_node : Node, path : steer.Path):
        node = start_node
        for segment in path.segments:
            path_new = steer.PathSimple(segment)
            new_edge = Edge(node, path_new)
            self.edges.append(new_edge)
            self.node_poses = np.append(self.node_poses, np.atleast_2d(new_edge.end_node.pose), axis = 0)
            self.node_distances = np.append(self.node_distances, new_edge.end_node.distance_from_origin)
            node = new_edge.end_node
        # print("Added new node")

    '''
    This function selects a random pose in the environment (which is not in colision) and connects it to the graph
    '''
    def grow_single(self):
        return self.add_path_to(self.map.random_pose())


    def connect_to_newest_node(self, new_pose : np.ndarray):
        path = steer.optimal_path(self.node_poses[-1], new_pose, self.turning_radius)
        discrete_path = path.interpolate(d=self.collision_resolution)
        collision = self.map.collision_check(discrete_path)
        if collision:
            return False
        # add path to tree
        if len(self.edges) > 0:
            self.add_node(self.edges[-1].end_node, path)
        else:
            self.add_node(self.base_node, path)
        return True


    '''
    This function finds a path from a node on the tree to the new node.
    If this path is in collision, it is not added to the tree.
    Using an upper bound on the shortest path to a node (dubbins path), most nodes can be ignored when generating dubbins paths.
    '''
    def add_path_to(self, new_pose : np.ndarray, modify_angle=True) -> bool:
        distances = np.linalg.norm(self.node_poses[:,:2] - new_pose[:2], axis=1) + self.node_distances
        closest_distance = np.min(distances)
        upper_bound = closest_distance + 7/3 * np.pi * self.turning_radius
        valid_indices = distances <= upper_bound
        valid_indices = np.arange(len(valid_indices))[valid_indices]

        potential_steering_paths : list[steer.Path] = []
        angle_random = new_pose[2]

        for idx in valid_indices:
            if modify_angle:
                displacement = (new_pose - self.node_poses[idx])[:2]
                angle_displacement = np.arctan2(displacement[1], displacement[0])
                angle = (angle_displacement + angle_random) % (np.pi * 2)
                new_pose[2] = angle
            potential_steering_paths.append(steer.optimal_path(self.node_poses[idx], new_pose, self.turning_radius))

        shortest_path_idxs = np.argsort([path.length + self.node_distances[valid_indices][i] for i, path in enumerate(potential_steering_paths)])
        
        for i, shortest_path_idx in enumerate(shortest_path_idxs):
            if i > 4:
                break

            steering_path = potential_steering_paths[shortest_path_idx]
            parent_coord_idx = valid_indices[shortest_path_idx]

            discrete_path = steering_path.interpolate(d=self.collision_resolution)

            collision = self.map.collision_check(discrete_path)
            if collision:
                continue

            if parent_coord_idx == 0:
                self.add_node(self.base_node, steering_path)
            else:
                self.add_node(self.edges[parent_coord_idx-1].end_node, steering_path)

            return True
        return False


# -----------------------------------------------------------------------------
# Define class that visualizes the RRT algorithm and final path
# -----------------------------------------------------------------------------

class RRTPlot():

    def __init__(self, start, goal, workspace_size, workspace_center, obstacles):

        # Define start position and orientation of the robot
        start_coords = to_pygame_coords(start[:2], workspace_size)
        self.start_x = meters2pixels(start_coords[0])
        self.start_y = meters2pixels(start_coords[1])
        self.start_theta = start[2]

        # Define goal position and orientation of the robot
        goal_coords = to_pygame_coords(goal[:2], workspace_size)
        self.goal_x = meters2pixels(goal_coords[0])
        self.goal_y = meters2pixels(goal_coords[1])
        self.goal_theta = goal[2]

        # Define workspace dimensions: length (x) and width (y)
        self.workspace_x = meters2pixels(workspace_size[0])
        self.workspace_y = meters2pixels(workspace_size[1])
        self.workspace_size = workspace_size

        # Define center of the workspace
        self.workspace_cx = meters2pixels(workspace_center[0])
        self.workspace_cy = meters2pixels(workspace_center[1])

        # Read the obstacle information
        self.obstacles = obstacles

        # Define some colours to be used
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)

        # Define the visuals of the workspace
        self.workspace = pygame.display.set_mode(size=(self.workspace_x, self.workspace_y))
        self.workspace.fill(self.white)
        self.nodeRad = 0
        self.nodeThickness = 0
        self.edgeThickness = 1

    def draw_workspace(self):
        # Draw start position of robot
        pygame.draw.circle(self.workspace, self.green, center = (self.start_x, self.start_y), radius = self.nodeRad+10.0)

        # Draw goal position of robot
        pygame.draw.circle(self.workspace, self.red, center = (self.goal_x, self.goal_y), radius = self.nodeRad+10.0)

        # Draw obstacles
        self.draw_obstacles()

    def draw_obstacles(self):

        for i in range(len(self.obstacles)):

            obstacle_coords = to_pygame_coords(self.obstacles[i,:2], self.workspace_size)
            obstacle_x = meters2pixels(obstacle_coords[0])
            obstacle_y = meters2pixels(obstacle_coords[1])
            obstacle_theta = self.obstacles[i, 2]
            obstacle_l = meters2pixels(self.obstacles[i, 3])
            obstacle_w = meters2pixels(self.obstacles[i, 4])

            # Tranform reference point from cenetr of the rectangle to top-left corner of rectangle
            obstacle_left = obstacle_x - obstacle_l/2
            obstacle_top = obstacle_y - obstacle_w/2

            pygame.draw.rect(self.workspace, self.black, pygame.Rect(obstacle_left, obstacle_top, obstacle_l, obstacle_w))

# -----------------------------------------------------------------------------
# Define some auxiliary functions
# -----------------------------------------------------------------------------

def meters2pixels(x):
   return x * 30

def to_pygame_coords(point, window_size):
    
    """
    This function coverts the given point coordinates which are given with 
    respect to the center of the window to the reference frame used in pygame.
    This pygame reference frame has its origin in the top left edge of the window
    """
    x_offset = window_size[0]/2
    y_offset = window_size[1]/2
    
    x = point[0]
    y = point[1]
    
    if y > 0:
        y_new = y_offset - y
    else:
        y_new = y_offset + abs(y)

    if x > 0:
        x_new = x_offset + x
    else:
        x_new = x_offset - abs(x)
    new_point = [x_new, y_new]
    return new_point


