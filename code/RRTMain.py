# -----------------------------------------------------------------------------
# Import needed packages
# -----------------------------------------------------------------------------

import sys

import time
import numpy as np

from tqdm import tqdm, trange
from sys import exit

import pickle

from matplotlib import pyplot as plt
from matplotlib import collections as mc
from matplotlib import patches

sys.path.append("../mujoco")

# Import from custom files
import RRT
import Steering as steer
import carenv
import Approximator
import CarController

# np.random.seed(428)


class consts():
    turning_radius = 0.8
    collision_resolution = 0.05
    point_resolution = 0.05
    vehicle_radius = 0.3
    lookahead_m = 3.0
    lookahead : int = int(lookahead_m // point_resolution)
    workspace_center = np.array([0, 0])
    workspace_size = np.array([30, 30])
    recompute_error_treshold = 2.0
    render_mode = 1 # 0 = render off, 1 = render on 2 = offscreen render (for video)

# -----------------------------------------------------------------------------
# Define main function
# -----------------------------------------------------------------------------

def main():
    # Deifne the start and end points
    start_pose = RRT.pose_deg(5.0, -8.0, -170)
    final_pose = RRT.pose_deg(-1.5, 9.25, 0)

    # Create environment and extract relevant information
    env = carenv.Car(mode=consts.render_mode)
    initial_pose, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) # start with reset

    # grow/load the tree
    tree = test_rrt_reverse(obstacles, grow=False, final_pose=final_pose)
    
    tree.lookahead = consts.lookahead
    
    tree.print()
    # ax = tree.plot()
    # RRT.plot_pose(ax, start_pose, color="green")
    # RRT.plot_pose(ax, final_pose, color="red")
    # plt.pause(0.1)
    
    # Run the simulation
    CarController.mujoco_sim(env, start_pose, tree, consts) 
    
    plt.show()

def test_rrt_reverse(obstacles, final_pose, grow=False):
    if grow:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose, itera=trange(10000), max_seconds=5*60)
        with open("tree3.pickle", "wb") as outfile:
            # "wb" argument opens the file in binary mode
            pickle.dump(tree, outfile)
    else:
        print("loading...")
        with open("tree3.pickle", "rb") as infile:
            tree : RRT.Tree = pickle.load(infile)
        print("loaded.")
    return tree
       
    
if __name__ == '__main__':
    main()
