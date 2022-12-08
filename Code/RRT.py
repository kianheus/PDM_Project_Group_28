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
        self.graph_x = [start_x]
        self.graph_y = [start_y]
        self.graph_parent = [0]


    def new_point(self):
        new_x = np.random.uniform(workspace_center_x - workspace_x/2, workspace_center_x + workspace_x/2)
        new_y = np.random.uniform(workspace_center_y - workspace_y/2, workspace_center_y + workspace_y/2)
        valid_point = self.PathCheck(new_x, new_y)
        pass

    def path_check(self, new_x, new_y):
        closest_distance = np.linalg.norm((workspace_x, workspace_y))
        for i, (x, y) in enumerate(zip(self.graph_x, self.graph_y)):
            node_distance = np.linalg.norm((new_x-x, new_y-y))
            if node_distance < closest_distance:
                closest_distance = node_distance
                closest_node_id = i
        self.graph_x.append(new_x)
        self.graph_y.append(new_y)
        self.graph_parent.append(closest_node_id)
        return()

def meters2pixels(x):
    return x * 100

class RRTPlot():
    
    def __init__(self, start_x, start_y, goal_x, goal_y, workspace_x, workspace_y, obstacles):
        self.start_x = meters2pixels(start_x)
        self.start_y = meters2pixels(start_y)
        
        self.goal_x = meters2pixels(goal_x)
        self.goal_y = meters2pixels(goal_y)
        
        self.workspace_x = meters2pixels(workspace_x)
        self.workspace_y = meters2pixels(workspace_y)
        
        self.obstacles = obstacles
        
        self.workspace = pygame.display.set_mode(size=(self.workspace_x, self.workspace_y))   
        self.workspace.fill((255, 255, 255))
        self.nodeRad = 0
        self.nodeThickness = 0
        self.edgeThickness = 1
        
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
                                                  
    def draw_workspace(self):
        pygame.draw.circle(self.workspace, self.green, center = (self.start_x, self.start_y), radius = self.nodeRad+10.0)
        pygame.draw.circle(self.workspace, self.red, center = (self.goal_x, self.goal_y), radius = self.nodeRad+10.0)
        self.draw_obstacles()
        
    def draw_obstacles(self):
        
        obstacles_x = meters2pixels(self.obstacles[:, 0])
        obstacles_y = meters2pixels(self.obstacles[:, 1])
        obstacles_theta = self.obstacles[:, 2]
        obstacles_l = meters2pixels(self.obstacles[:, 3])
        obstacles_w = meters2pixels(self.obstacles[:, 4])
        
        print(obstacles_x[1].shape)
        print(obstacles_x[1])
        
        for i in range(len(self.obstacles)):
            pygame.draw.rect(self.workspace, self.black, pygame.Rect(obstacles_x[i], obstacles_y[i], obstacles_l[i], obstacles_w[i]))
        
    

workspace_center_x = 0
workspace_center_y = 0

workspace_x = 20
workspace_y = 10

start_x = 2
start_y = 2
start_theta = 0

goal_x = 18
goal_y = 8

# [x, y, rotation, length, width]
obstacles = np.array([[5, 5, 0, 2, 2], [8, 8, 0, 2, 2]]) 


pygame.init()
workspace = RRTPlot(start_x, start_y, goal_x, goal_y, workspace_x, workspace_y, obstacles)
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
for i in range(5000):
    if i%100 
    RRT_calculator.NewPoint()



lines = []
for i, (x, y, parent) in enumerate(zip(RRT_calculator.graph_x, RRT_calculator.graph_y, RRT_calculator.graph_parent)):
    line = [(x, y), (RRT_calculator.graph_x[parent], RRT_calculator.graph_y[parent])]
    lines.append(line)
lc = mc.LineCollection(lines, linewidths=2)


fig, ax = plt.subplots()
ax.set_xlim(-10, 10)
ax.set_ylim(-5, 5)
ax.add_collection(lc)
print(time.time() - start_time)
plt.show()

#plt.scatter(RRT_calculator.graph_x, RRT_calculator.graph_y)
#plt.show()