# -----------------------------------------------------------------------------
# Import needed packages
# -----------------------------------------------------------------------------

import pygame

import sys
sys.path.append("../mujoco")
sys.path.append("mujoco")

import carenv
import time
import numpy as np

from tqdm import tqdm, trange
from sys import exit
from RRT import RRTPlot
import RRT
import Steering as steer
from matplotlib import pyplot as plt
from matplotlib import collections as mc
from matplotlib import patches


# -----------------------------------------------------------------------------
# Define main fucntion
# -----------------------------------------------------------------------------

def main():
    
    # Define code to execute, can be "Paula" or "Kian" or "Thomas"
    user = "Thomas"
    
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


    if user == "Thomas":
        env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)

        initial_pose = RRT.pose_deg(0.0, 0.0, 0)
        final_pose = RRT.pose_deg(2.0, -4.0, 180)

        tree = RRT.Tree(env_map, turning_radius=radius, initial_pose=initial_pose, collision_resolution=0.05)
        done = False
        close_time=time.time() + 60
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
    

if __name__ == '__main__':
    main()

