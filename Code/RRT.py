import numpy as np
import math
import pygame
import time
from sys import exit
from matplotlib import pyplot as plt
from matplotlib import collections as mc

start_time = time.time()
class RRTCalc:

    def __init__(self, start_x, start_y):
        #self.graph_x = np.array([start_x])
        #self.graph_y = np.array([start_y])
        self.graph_coords = np.array([[start_x, start_y]])
        self.graph_parent = np.array([0])


    def new_point(self):
        new_x = np.random.uniform(workspace_center_x - workspace_x/2, workspace_center_x + workspace_x/2)
        new_y = np.random.uniform(workspace_center_y - workspace_y/2, workspace_center_y + workspace_y/2)
        new_coord = np.array([new_x, new_y])
        valid_point = self.path_check(new_coord)
        pass

    def path_check(self, new_coord):
        closest_node_id = np.argmin(np.linalg.norm(self.graph_coords - new_coord, axis=1))
        self.graph_coords = np.append(self.graph_coords, np.array([new_coord]), axis = 0)
        self.graph_parent = np.append(self.graph_parent, closest_node_id)
        return()

def meters2pixels(x):
    return x * 100

def top_left_coodinates(point, size):
    u = size/2
    point_new_x = u[0] + point[0]
    point_new_y = u[1] - point[1]
    new_point = np.array([point_new_x, point_new_y])
    return(new_point)

class RRTPlot():
    
    def __init__(self, start, goal, workspace_size, workspace_center, obstacles):
        # Define start position and orientation of the robot
        start_coords = meters2pixels(top_left_coodinates(start[:2], workspace_size))
        self.start_x = start_coords[0]
        self.start_y = start_coords[1]
        self.start_theta = start[2]
        
        # Define goal position and orientation of the robot  
        goal_coords = meters2pixels(top_left_coodinates(goal[:2], workspace_size))
        self.goal_x = goal_coords[0]
        self.goal_y = goal_coords[1]
        self.goal_theta = goal[2]
        
        # Define workspace dimensions: length (x) and width (y)
        self.workspace_x = meters2pixels(workspace_size[0])
        self.workspace_y = meters2pixels(workspace_size[1])
        
        # Define center of the workspace 
        self.workspace_cx = meters2pixels(workspace_center[0])
        self.workspace_cy = meters2pixels(workspace_center[1])
        
        # Read the obstacle information
        self.obstacles = obstacles

        # Define some colour to be used
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
        
        
        obstacle_coords = meters2pixels(top_left_coodinates(self.obstacles[:, :2], workspace_size))       
        obstacles_x = obstacle_coords[:,0]
        obstacles_y = obstacle_coords[:,1]
        obstacles_theta = self.obstacles[:, 2]
        obstacles_l = meters2pixels(self.obstacles[:, 3])
        obstacles_w = meters2pixels(self.obstacles[:, 4])
        
        #obs_left = obstacles_x - obstacles_l/2
        #obs_top = obstacles_y + obstacles_w/2
        
        for i in range(len(self.obstacles)):
            pygame.draw.rect(self.workspace, self.black, pygame.Rect(obstacles_x[i], obstacles_y[i], obstacles_l[i], obstacles_w[i]))
        
    
# Define some sets of cooridnates
workspace_center = np.array([0, 0]) # Coordinate center of workspace
workspace_size = np.array([8, 8]) # Dimensions of workspace 
start = np.array([0, 0, 0]) # Starting position and orientation of robots (x, y, theta)
goal = np.array([3, 3, 0]) # Goal position and orientation of robot (x, y, theta)

# [x, y, rotation, length, width]
obstacles = np.array([[0, 0, 0, 1, 0.5], [6, 6, 0, 1, 0.5]]) 

pygame.init()
workspace = RRTPlot(start, goal, workspace_size, workspace_center, obstacles)
workspace.draw_workspace()
pygame.display.update()
pygame.event.clear()
pygame.event.wait(0)


while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
"""

RRT_calculator = RRTCalc(start_x, start_y)
for i in range(10000):
    if i%100 == 0:
        print(i)
    RRT_calculator.new_point()



lines = []
for i, (coords, parent) in enumerate(zip(RRT_calculator.graph_coords, RRT_calculator.graph_parent)):
    line = [RRT_calculator.graph_coords[i,:], (RRT_calculator.graph_coords[parent, :])]
    lines.append(line)
lc = mc.LineCollection(lines, linewidths=2, cmap='jet')


fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-5, 5)
ax.add_collection(lc)
plt.show()

#plt.scatter(RRT_calculator.graph_x, RRT_calculator.graph_y)
#plt.show()
"""