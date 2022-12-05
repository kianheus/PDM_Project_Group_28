import numpy as np
import math
import pygame

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





class RRT_plot():

    def __init__(self, start_x, start_y, goal_x, goal_y, workspace_x, workspace_y, obstacles)
        self.start_x = start_x
        self.start_y = start_y
        
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.workspace_x = workspace_x
        self.workspace_y = workspace_y
    
    def DrawMap():
        pass
    

workspace_x = 20
workspace_y = 10

start_x = 2
start_y = 2
start_theta = 0

goal_x = 18
goal_y = 8



