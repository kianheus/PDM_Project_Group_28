import numpy as np
import math
import pygame
from sys import exit

class RRTCalc(self, start_x, start_y):


    self.graph.x = [start_x]
    self.graph.y = [start_y]
    self.graph.parent = [0]



    def Method():
        pass

    def NewPoint():
        new_x = np.random.uniform(0, workspace_x)
        new_y = np.random.uniform(0, workspace_y)
        valid_point = PathCheck(new_x, new_y)
        pass

    def PathCheck(new_x, new_y):
        for i, (x, y) in enumerate(zip(self.graph.x, self.graph.y)):
            print(i, x, y, new_x, new_y)
        return True





class RRTPlot():

    def __init__(self, start_x, start_y, goal_x, goal_y, workspace_x, workspace_y):
        self.start_x = int(start_x)
        self.start_y = int(start_y)
        
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.workspace_x = workspace_x
        self.workspace_y = workspace_y
        
        self.workspace = pygame.display.set_mode(size=(self.workspace_x, self.workspace_y))   
        self.workspace.fill((255, 255, 255))
        self.nodeRad = 0
        self.nodeThickness = 0
        self.edgeThickness = 1
        
        self.green = (0, 255, 0)
        self.red = (255, 0, 0)
        self.white = (255, 255, 255)
        self.black = (0, 0, 0)
                                          
    def drawWorkspace(self):
        pygame.draw.circle(self.workspace, self.black, center = (self.start_x, self.start_y), radius = self.nodeRad+5.0)
        pygame.draw.circle(self.workspace, self.black, center = (self.goal_x, self.goal_y), radius = self.nodeRad+5.0)
        
    

workspace_x = 600
workspace_y = 1000

start_x = 50
start_y = 50
start_theta = 0

goal_x = 500
goal_y = 500

pygame.init()
workspace = RRTPlot(start_x, start_y, goal_x, goal_y, workspace_x, workspace_y)
workspace.drawWorkspace()
pygame.display.update()
pygame.event.clear()
pygame.event.wait(0)

while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
                


