"""
----------------------------------------------------------------------------------------
This is the main script from which other parts of the code are run. 

This file is structured as follows:
1. Imports
2. Definition of important constants
3. Main function, which creates an environment, runs RRT code to create a path, 
   simulates the found path in MuJoCo and outputs results.
4. Function to develop RR tree backwards, from goal to start position
5. Implementation of MuJoCo for this project
-----------------------------------------------------------------------------------------
"""

# Import needed packages
import numpy as np
from tqdm import trange
import pickle

import sys

sys.path.append("../mujoco")

# Import from custom files
import RRT
import carenv
import CarController
import struct

def create_unique_number(pose):
    data = struct.pack('>3f', pose[0], pose[1], pose[2])
    hex_data = data.hex()
    return hex_data

class consts():
    turning_radius = 0.8 #[m]
    collision_resolution = 0.05 #[m]
    point_resolution = 0.05 #[m]
    vehicle_radius = 0.3 #[m]
    lookahead_m = 3.0 #[m]
    lookahead : int = int(lookahead_m // point_resolution) #[-]
    workspace_center = np.array([0, 0]) #[m,m]
    workspace_size = np.array([30, 30]) #[m,m]
    recompute_error_treshold = 2.0 #[m]
    render_mode = 1 # 0 = render off, 1 = render on 2 = offscreen render (for video)

# -----------------------------------------------------------------------------
# Define main function
# -----------------------------------------------------------------------------

def main():
    # Define the start and end points and angles
    start_pose = RRT.pose_deg(4.0, -7.0, 90)
    final_pose = RRT.pose_deg(-1.5, 9.25, 0)

    # Create environment and extract relevant information
    env = carenv.Car(mode=consts.render_mode)
    initial_pose, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) # start with reset

    # grow/load the tree
    tree = test_rrt_reverse(obstacles, grow=True, final_pose=final_pose)
    tree.lookahead = consts.lookahead
    tree.print()    
    
    # Run the simulation
    CarController.mujoco_sim(env, start_pose, tree, consts) 

def test_rrt_reverse(obstacles, final_pose, grow=False):
    """
    number = create_unique_number(final_pose)
    filename = "../trees/" + str(number) + ".pickle"
    try:
        with open(filename, "rb") as infile:
            tree : RRT.Tree = pickle.load(infile)
    except FileNotFoundError:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose, itera=trange(10000), max_seconds=5*60)
        with open(filename, "wb") as outfile:
            # "wb" argument opens the file in binary mode
            pickle.dump(tree, outfile)
    
    """
    if grow:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose, itera=trange(10000), max_seconds=5*60)
        with open("tree4.pickle", "wb") as outfile:
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
