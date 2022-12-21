import numpy as np
import math
import pygame
import time
from sys import exit
from matplotlib import pyplot as plt
from matplotlib import collections as mc
from matplotlib import patches

import carenv
env = carenv.Car(render=False)
state, obstacles = env.reset() #start with reset
obstacles[:,3:] = obstacles[:,3:]*2 

start_time = time.time()
class RRTCalc:

    def __init__(self, start_x, start_y, obstacles, n_line_segments = 100):
        self.graph_coords = np.array([[start_x, start_y]])
        self.graph_parent = np.array([0])
        self.n_line_segments = n_line_segments
        self.obstacles = obstacles
        self.colours = np.array([["blue"]])


    def new_point(self):
        collision = True
        while collision:
            new_coord = np.random.uniform(low=workspace_center-workspace_size/2, high=workspace_center+workspace_size/2, size = (1,2))[0]
            collision = self.collision_check(new_coord)
        self.path_check(new_coord)

    def path_check(self, new_coord):
        closest_node_id = np.argmin(np.linalg.norm(self.graph_coords - new_coord, axis=1))
        parent_coord = self.graph_coords[closest_node_id]
        discrete_path = self.path_discretization(new_coord, parent_coord)
        collision = False
        for i in range(len(discrete_path)):
            collision = self.collision_check(discrete_path[i])
            if collision:
                break
        """
        if not collision:
            self.colours = np.append(self.colours, np.array(["blue"]))
        else:
            self.colours = np.append(self.colours, np.array(["red"]))
        """
        if not collision:
            self.graph_coords = np.append(self.graph_coords, np.array([new_coord]), axis = 0)
            self.graph_parent = np.append(self.graph_parent, closest_node_id)
        return()

    def collision_check(self, point):
        for obstacle in self.obstacles:
            if point[0] > obstacle[0] - obstacle[3]/2 and point[0] < obstacle[0] + obstacle[3]/2\
                and point[1] > obstacle[1] - obstacle[4]/2 and point[1] < obstacle[1] + obstacle[4]/2:
                return(True)
                break
        return(False)

    def path_discretization(self, new_coord, parent_coord):
        return(np.array([np.linspace(parent_coord[0], new_coord[0], self.n_line_segments),
                   np.linspace(parent_coord[1], new_coord[1], self.n_line_segments)]).T)



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
#obstacles = np.array([[0, 0, 0, 1, 1.5], [-3, -3, 0, 1, 0.5]])

user = "Paula"


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
    RRT_calculator = RRTCalc(start_coord[0], start_coord[1], obstacles)
    for i in range(2000):
        if i%100 == 0 and i>0:
            print(i)
        RRT_calculator.new_point()



    lines = []
    for i, (coords, parent) in enumerate(zip(RRT_calculator.graph_coords, RRT_calculator.graph_parent)):
        line = [RRT_calculator.graph_coords[i,:], (RRT_calculator.graph_coords[parent, :])]
        lines.append(line)
    lc = mc.LineCollection(lines, linewidths=2)#, colors = RRT_calculator.colours)



    fig, ax = plt.subplots()
    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    ax.add_collection(lc)
    for i in range(obstacles.shape[0]):
        ax.add_patch(patches.Rectangle((obstacles[i][0]-obstacles[i][3]/2,obstacles[i][1]-obstacles[i][4]/2), obstacles[i][3], obstacles[i][4]))
    plt.show()

    #plt.scatter(RRT_calculator.graph_x, RRT_calculator.graph_y)
    #plt.show()
