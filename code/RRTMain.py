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
import struct

sys.path.append("../mujoco")

# Import from custom files
import RRT # everything related to the path
import carenv # physics based environment and graphics
import CarController # controlling the car based on path and apply it to the environment

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
    start_pose = RRT.pose_deg(4.0, -7.0, 90) #[m, m, deg]
    final_pose = RRT.pose_deg(-1.5, -9.25, 0) #[m, m, deg]
    
# all grown trees
# RRT.pose_deg(-1.5, -9.25, 0)
# RRT.pose_deg(-1.5, 9.25, 0)

# -----------------------------------------------------------------------------
# Define main function
# -----------------------------------------------------------------------------

def main():

    # Create environment and extract relevant information
    env = carenv.Car(mode=consts.render_mode)
    initial_pose, obstacles, moving_obstacles = env.reset(consts.start_pose) # start with reset

    # grow/load the tree
    tree = load_grow_tree(obstacles, final_pose=consts.final_pose)
    tree.lookahead = consts.lookahead
    tree.print()    
    
    # Run the simulation
    CarController.mujoco_sim(env, consts.start_pose, tree, consts) 

# checks final position if there is already a tree grown for it, otherwise grow tree and save to file
def load_grow_tree(obstacles, final_pose):
    
    #extract unique number based on the final_pose
    number = create_unique_number(final_pose)
    
    # file name used for grown tree files
    filename = "../trees/" + str(number) + ".pickle"
    try:
        with open(filename, "rb") as infile:
            tree : RRT.Tree = pickle.load(infile)
    except FileNotFoundError:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose, itera=trange(10000), max_seconds=5*60)
        with open(filename, "wb") as outfile:
            # "wb" argument opens the file in binary mode
            pickle.dump(tree, outfile)
    return tree

# creates an unique number based on the final pose
def create_unique_number(pose):
    data = struct.pack('>3f', pose[0], pose[1], pose[2])
    hex_data = data.hex()
    return hex_data
    
if __name__ == '__main__':
    main()
