# -----------------------------------------------------------------------------
# Import needed packages
# -----------------------------------------------------------------------------

import pygame
import carenv
import time
import numpy as np

from tqdm import tqdm
from sys import exit
from RRT import RRTPlot
from RRT import RRTCalc
from matplotlib import pyplot as plt
from matplotlib import collections as mc
from matplotlib import patches


# -----------------------------------------------------------------------------
# Define main fucntion
# -----------------------------------------------------------------------------

def main():
    
    # Define code to execute, can be "Paula" or "Kian"
    user = "Kian"
    
    # Create environment and extract relevant information
    env = carenv.Car(render=False)
    state, obstacles = env.reset() #start with reset
    obstacles[:,3:] = obstacles[:,3:]*2 # [x, y, rotation, length, width]

    start_time = time.time()
    
    # Define some sets of cooridnates
    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([30, 30]) # Dimensions of workspace
    start_coord = state # Starting position and orientation of robot (x, y, theta)
    goal_coord = np.array([0, 10.05, 0]) # Goal position and orientation of robot (x, y, theta)
    
    #Computational variables
    n_line_segments = 100
    radius = 0.5
    collision_resolution = 0.05
    
    
    
    
    if user == "Paula":
        pygame.init()
        workspace = RRTPlot(start_coord,
                            goal_coord,
                            workspace_size,
                            workspace_center,
                            obstacles)
        
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
        RRT_calculator = RRTCalc(start_coord[0],
                                 start_coord[1],
                                 start_coord[2],
                                 obstacles,
                                 collision_resolution,
                                 radius,
                                 workspace_center, 
                                 workspace_size,
                                 vehicle_radius=0.1)
        
        for i in tqdm(range(30)): #range(200): #
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

if __name__ == '__main__':
    main()

