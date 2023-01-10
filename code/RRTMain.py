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


# np.random.seed(428)


class consts():
    turning_radius = 0.8
    collision_resolution = 0.05
    point_resolution = 0.05
    vehicle_radius = 0.3
    lookahead_m = 5.0
    lookahead : int = int(lookahead_m // point_resolution)
    workspace_center = np.array([0, 0])
    workspace_size = np.array([30, 30])


# -----------------------------------------------------------------------------
# Define main function
# -----------------------------------------------------------------------------

def main():
    # Deifne the start and end points
    start_pose = RRT.pose_deg(-3.5, 9.25, 180)
    final_pose=RRT.pose_deg(3.5, 5.0, 180)
    
    # Create environment and extract relevant information
    env = carenv.Car(render=True)
    initial_pose, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) # start with reset

    # grow/load the tree
    tree = test_rrt_reverse(obstacles, grow=False)
    
    # Run the simulation
    mujoco_sim(env, start_pose, tree)


def test_rrt_reverse(obstacles, grow=False, final_pose=RRT.pose_deg(3.5, -5.0, 180)):
    if grow:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose)
        with open("tree.pickle", "wb") as outfile:
            # "wb" argument opens the file in binary mode
            pickle.dump(tree, outfile)
    else:
        print("loading...")
        with open("tree.pickle", "rb") as infile:
            tree : RRT.Tree = pickle.load(infile)
        print("loaded.")
    return tree


# Fabio    

def bound(low, high, value):
     return max(low, min(high, value))
            
def mujoco_sim(env, start_pose, tree):
    #function for PID controller
    class PIDcontroller():

        def __init__(self, kp, ki, kd):
            #initialise P, I, D factors
            self.kp = kp
            self.ki = ki
            self.kd = kd
            self.pe = 0
            self.se = 0
            self.dt = 0.01 #loop runs at 100 Hz

        def pid(self, e):

            self.se = self.se + e*self.dt # intergrated error

            P = self.kp*e
            I = self.ki*self.se # forward euler
            D = self.kd*(e-self.pe)/self.dt # forward difference

            output = P + I + D # output of pid controller

            self.pe = e # previous error used in differentiation

            return output

    # initialise pid controllers
    theta_pid = PIDcontroller(1, 0, 0)
    lateral_pid = PIDcontroller(1, 0, 0)
    longitudal_pid = PIDcontroller(15, 0, 3)

    state, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) #start with reset
    
    start_pose_rev = steer.reverse_pose(state.copy())
    tree.add_path_to(start_pose_rev, modify_angle=False, n_closest=100, i_break=25)
    path = tree.path_to(start_pose_rev)
    if path is None:
        print("no path found")
        return
    points = path.interpolate_poses(d=consts.point_resolution, reverse=True)

    original_points = points.copy()

    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([30, 30]) # Dimensions of workspace
    env_map = RRT.Map(obstacles, consts=consts)
    starttime = time.time()
    i = 0
    n = 0
    states = np.expand_dims(state.copy(), axis=0)
    #simulate for 100 s
    while True:
        ########################## desired goals generated by path planner ########################
        thetar = points[i,2]# desired heading angle for following path
        pr = points[i, 0:2] #desired point where car needs to go

        ########################## heading + longitudal + lateral error ##########################
        #heading
        theta = state[2]
        theta_error = ((thetar-theta+np.pi) % (2*np.pi)) - np.pi #multiple rotations without sudden gap between 0 and 2pi

        #print(obstacles)
        #longitudal + lateral
        p = np.array([state[0], state[1]])
        distance = np.linalg.norm(pr - p) #total distance
        angle = np.arctan2(p[1]-pr[1],p[0]-pr[0]) #subtract reference
        difference_angle = thetar - angle

        lateral_error = -np.sin(difference_angle)*distance
        longitudal_error = np.cos(difference_angle)*distance

        ########################## PID ##########################
        steering_angle_theta = theta_pid.pid(theta_error)
        steering_angle_lateral = -lateral_pid.pid(lateral_error)
        throttle = longitudal_pid.pid(longitudal_error)

        ########################## Bounds and combining output of controlllers ##########################
        #combine steering input
        steering_angle = steering_angle_lateral + steering_angle_theta

        #bounds
        throttle = bound(-5, 5, throttle)
        steering_angle = bound(-0.38, 0.38, steering_angle)

        ########################## sim ##########################
        action = np.array([steering_angle,-throttle])  #action: first numer [-0.38, 0.38] - = right, + = left. Second number [unconstrained] - backward. + = forwar
        state, obstacles, moving_obstacles = env.step(action) #set step in environment
        env.render(mode = True) # turn rendering on or off
        states = np.vstack((states, state))

        ########################## reset after 10 seconds, this can be changed ##########################

        if env.get_time() > 20:
            env.close_window()
            break

        
        if n % 2 == 0:
            i = i + 1
            i = bound(0, points.shape[0]-1, i)
            
        if n % 10 == 0:
            points2, reroute = tree.local_planner(state, obstacles, moving_obstacles, points, i)
            if reroute == True:
                points = points2
                i = 0

        n = n+1
        
        #time.sleep(0.01 - ((time.time() - starttime) % 0.01)) # sleep for 100 Hz realtime loop
    fig, ax = plt.subplots()
    env_map.plot(ax)
    ax.scatter(states[:,0], states[:,1], c=range(states.shape[0]), cmap='viridis')
    ax.plot(original_points[:,0], original_points[:,1], color='red')
    ax.axis("equal")
    plt.show()

     
    
if __name__ == '__main__':
    main()
