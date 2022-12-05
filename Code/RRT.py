import numpy as np
import math
import pygame
from sys import exit
from matplotlib import pyplot as plt
from matplotlib import collections as mc
class RRTCalc:

    def __init__(self, start_x, start_y):
        self.graph_x = [start_x]
        self.graph_y = [start_y]
        self.graph_parent = [0]



    def Method(self):
        pass

    def NewPoint(self):
        new_x = np.random.uniform(workspace_center_x - workspace_x/2, workspace_center_x + workspace_x/2)
        new_y = np.random.uniform(workspace_center_y - workspace_y/2, workspace_center_y + workspace_y/2)
        valid_point = self.PathCheck(new_x, new_y)
        pass

    def PathCheck(self, new_x, new_y):
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

def MetersToPixel(x):
    return x * 100

class RRTPlot():
    
    def __init__(self, start_x, start_y, goal_x, goal_y, workspace_x, workspace_y, obstacles):
        self.start_x = MetersToPixel(start_x)
        self.start_y = MetersToPixel(start_y)
        
        self.goal_x = MetersToPixel(goal_x)
        self.goal_y = MetersToPixel(goal_y)
        
        self.workspace_x = MetersToPixel(workspace_x)
        self.workspace_y = MetersToPixel(workspace_y)
        
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
                                                  
    def DrawWorkspace(self):
        pygame.draw.circle(self.workspace, self.green, center = (self.start_x, self.start_y), radius = self.nodeRad+10.0)
        pygame.draw.circle(self.workspace, self.red, center = (self.goal_x, self.goal_y), radius = self.nodeRad+10.0)
        self.DrawObs()
        
    def DrawObs(self):
        
        obstacles_x = MetersToPixel(self.obstacles[:, 0])
        obstacles_y = MetersToPixel(self.obstacles[:, 1])
        obstacles_theta = self.obstacles[:, 2]
        obstacles_l = MetersToPixel(self.obstacles[:, 3])
        obstacles_w = MetersToPixel(self.obstacles[:, 4])
        
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
workspace.DrawWorkspace()
pygame.display.update()
pygame.event.clear()
pygame.event.wait(0)

while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
                


RRT_calculator = RRTCalc(start_x, start_y)
for i in range(500):
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
plt.show()
#plt.scatter(RRT_calculator.graph_x, RRT_calculator.graph_y)
#plt.show()