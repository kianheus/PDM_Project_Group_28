import numpy as np
import math
import pygame
import time
from sys import exit
from matplotlib import pyplot as plt
from matplotlib import collections as mc
from matplotlib import patches
import Steering as steer
from tqdm import tqdm, trange

user = "thomas"

import sys
sys.path.append("../mujoco")
sys.path.append("mujoco")

import carenv

env = carenv.Car(render=False)
state, obstacles = env.reset() #start with reset
obstacles[:,3:] = obstacles[:,3:]*2 


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
    def __init__(self, obstacles : np.ndarray, vehicle_radius : float):
        self.obstacles = obstacles
        self.vehicle_radius = vehicle_radius

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
            new_xy = np.random.uniform(low=workspace_center-workspace_size/2, high=workspace_center+workspace_size/2, size = (1,2))[0]
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
        self.add_path_to(self.map.random_pose())


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




def meters2pixels(x):
    return x * 30

def to_pygame_coords(point, window_size):
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


# Define some sets of cooridnates
workspace_center = np.array([0, 0]) # Coordinate center of workspace
workspace_size = np.array([30, 30]) # Dimensions of workspace
start_coord = state
#start_coord = np.array([0, 0, 0]) # Starting position and orientation of robots (x, y, theta)
goal_coord = np.array([0, 10.05, 0]) # Goal position and orientation of robot (x, y, theta)

#Computational variables
n_line_segments = 100

# [x, y, rotation, length, width]
# obstacles = np.array([[0, 0, 0, 1, 1.5], [-3, -3, 0, 1, 0.5]])
turning_radius = 0.5
collision_resolution = 0.1




if user == "Paula":
    pygame.init()
    workspace = RRTPlot(start_coord, goal_coord, workspace_size, workspace_center, obstacles)
    workspace.draw_workspace()
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type==pygame.KEYDOWN:
                if event.key==pygame.K_c:
                    intro=False
                if event.key==pygame.K_q:
                    pygame.quit()
                    exit()

if user == "Kian":
    RRT_calculator = RRTCalc(start_coord[0], start_coord[1], start_coord[2], obstacles, collision_resolution, turning_radius, vehicle_radius=0.1)
    for i in tqdm(range(3000)): #range(200): #
        RRT_calculator.new_point()


    """
    lines = []
    for i, (coords, parent) in enumerate(zip(RRT_calculator.graph_coords, RRT_calculator.graph_parent_idx)):
        line = [RRT_calculator.graph_coords[i,:2], (RRT_calculator.graph_coords[parent,:2])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=2)#, colors = RRT_calculator.colours)
    """

    fig, ax = plt.subplots()
    for path in RRT_calculator.steering_paths:
        path.plot(ax, endpoint=True, color="red", linewidth=1, alpha=0.8)
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    plt.axis("equal")
    #ax.add_collection(lc)
    for i in range(obstacles.shape[0]):
        ax.add_patch(patches.Rectangle((obstacles[i][0]-obstacles[i][3]/2,obstacles[i][1]-obstacles[i][4]/2), obstacles[i][3], obstacles[i][4]))
    plt.show()

    #plt.scatter(RRT_calculator.graph_x, RRT_calculator.graph_y)
    #plt.show()

if user == "thomas":
    env_map = Map(obstacles, 0.1)

    initial_pose = pose_deg(0.0, 0.0, 0)
    final_pose = pose_deg(2.0, -4.0, 180)

    tree = Tree(env_map, turning_radius=turning_radius, initial_pose=initial_pose, collision_resolution=0.05)
    done = False
    close_time=time.time() + 180
    for i in trange(1000):
        if time.time()>close_time:
            print("Time limit met, stopping.")
            break
        tree.grow_single()
        done = tree.add_path_to(final_pose, modify_angle=False)
        if done:
            print("Found a path.")
            break

    
    
    fig, ax = plt.subplots()
    env_map.plot(ax)

    for edge in tree.edges:
        # print("edge")
        edge.path.plot(ax, endpoint=True, color="orange", linewidth=1, alpha=0.3, s=0.4)

    if done:
        print("Done!")
        node = tree.edges[-1].end_node
        print(f"distance = {node.distance_from_origin:.02f}")
        dist = 0
        while node is not None:
            # edge.path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0)
            # steer.plot_point(ax, node.pose[:2], node.pose[2], color="orange")
            if node.parent_edge is not None:
                path : steer.Path = node.parent_edge.path
                dist += path.length
                path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
            node = node.parent_node

        print(f"distance = {dist:.02f}")
        

    steer.plot_point(ax, initial_pose[:2], initial_pose[2], color="green")
    steer.plot_point(ax, final_pose[:2], final_pose[2], color="red")

    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    plt.axis("equal")

    
    plt.show()


if user == "thomas2":
    print("aarg")
    env_map = Map(obstacles, 0.1)

    shape = (31, 31, 31)

    xx = np.linspace(-10, 10, shape[0])
    yy = np.linspace(-10, 10, shape[1])
    tt = np.linspace(0, 360, shape[2])

    z = np.zeros(shape)

    for x in trange(len(xx)):
        for y in range(len(yy)):
            for t in range(len(tt)):
                path = steer.optimal_path(pose_deg(0, 0, 0), pose_deg(xx[x], yy[y], tt[t]), 1.0)
                z[x,y,t] = path.length if path is not None else 0.0

    # zz = [steer.optimal_path(pose_deg(0, 0, 0), pose_deg(i, j, k), 1.0) for k in tqdm(t) for j in y for i in x]
    # z = np.array([path.length if path is not None else 0.0 for path in zz])

    X, Y = np.meshgrid(xx, yy)
    # Z = z.reshape(21, 21, 21)

    print(f"{X.shape=}")
    print(f"{Y.shape=}")
    print(f"{z.shape=}")

    fig, axs = plt.subplots(1, 1)

    c = axs.pcolor(X, Y, z[:,:,0].T)
    fig.colorbar(c, ax=axs)
    # plt.axes("equal")

    err = []
    errx = []

    for _ in range(10):
        pose = env_map.random_pose()
        length = steer.optimal_path(pose_deg(0, 0, 0), pose, 1.0).length

        
        x_index_1 = np.argmax((np.array(xx)-pose[0]) > 0)
        x_index_0 = x_index_1 - 1

        y_index_1 = np.argmax((np.array(yy)-pose[1]) > 0)
        y_index_0 = y_index_1 - 1

        t_index_1 = np.argmax((np.deg2rad(np.array(tt))-pose[2]) > 0)
        t_index_0 = t_index_1 - 1
        
        # print(f"{diff_x > 0}")
        # print(f"{diff_x < 0}")

        val_max = -np.inf
        for x_index in [x_index_0, x_index_1]:
            for y_index in [y_index_0, y_index_1]:
                for t_index in [t_index_0, t_index_1]:
                    val = z[x_index, y_index, t_index]
                    if val > val_max:
                        val_max = val
                    # print(f"{val=}")

        # print(f"upper bound =  {val_max}")
        # print(f"upper bound =  {np.linalg.norm(pose[:2]) + 7*np.pi/3 * turning_radius}")
        # print(f"actual value = {length}")

        err.append(val_max - length)
        errx.append(np.linalg.norm(pose[:2]) + 7*np.pi/3 * turning_radius - length)
        
        print(f"{pose=}: {length=}, {val_max=}")
        

        # print(f"{xx[x_index]}, {yy[y_index]}, {tt[t_index]}")
        # print(f"{xx[x_index]}, {yy[y_index]}, {tt[t_index]}")
        # print(f"x is in: ({xx[x_index_0]}, {xx[x_index_1]})")
        # print(f"y is in: ({yy[y_index_0]}, {yy[y_index_1]})")
        # print(f"t is in: ({tt[t_index_0]}, {tt[t_index_1]})")

    print(f"{err=}")
    print(f"{errx=}")



    



    # plt.show()

    # plt.imshow(Z,origin='lower',interpolation='bilinear')
    # plt.show()

    # for m in mg.T:
    #     # print(f"{m=}")
    #     for g in m:
    #         print(f"{g=}")