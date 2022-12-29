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
    radius = 0.5
    collision_resolution = 0.05
    
    # test_pygame(start_coord, goal_coord, workspace_size, workspace_center, obstacles)
    points = test_rrt(obstacles, workspace_center, workspace_size, radius, collision_resolution)
        
    
# Kian, Thomas
def test_rrt(obstacles, workspace_center, workspace_size, turning_radius, collision_resolution):
    env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)

    initial_pose = RRT.pose_deg(0.0, 0.0, 0)
    final_pose = RRT.pose_deg(7.0, 1.1, 0)

    tree = RRT.Tree(env_map, turning_radius=turning_radius, initial_pose=initial_pose, collision_resolution=collision_resolution)
    done = False
    close_time=time.time() + 30
    for i in trange(500):
        if time.time()>close_time:
            print("Time limit met, stopping.")
            break
        tree.grow_single()
        done = tree.add_path_to(final_pose, modify_angle=False) # Thomas, you idiot. We only need to check the newest node.
        if done:
            print("Found a path.")
            break

    
    
    fig, ax = plt.subplots()
    env_map.plot(ax)

    for edge in tree.edges:
        # print("edge")
        edge.path.plot(ax, endpoint=True, color="orange", linewidth=1, alpha=0.3, s=0.4)


    points = np.array([[0.0, 0.0, 0.0]])
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
                print(f"{points.shape=}")
                print(f"{path.interpolate_angles_2(d=0.05).shape=}")

                points = np.vstack((points, path.interpolate_angles_2(d=0.05)))
            node = node.parent_node
        points = points[1:,:]
        print(f"{points=}")

        print(f"distance = {dist:.02f}")

        plt.scatter(points[:,0], points[:,1])
        

    steer.plot_point(ax, initial_pose[:2], initial_pose[2], color="green")
    steer.plot_point(ax, final_pose[:2], final_pose[2], color="red")

    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    plt.axis("equal")

    plt.show()

    return points


# Paula
def test_pygame(start_coord, goal_coord, workspace_size, workspace_center, obstacles):
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



if __name__ == '__main__':
    main()

