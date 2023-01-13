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
    lookahead_m = 3.0
    lookahead : int = int(lookahead_m // point_resolution)
    workspace_center = np.array([0, 0])
    workspace_size = np.array([20, 20])
    recompute_error_treshold = 2.0


# -----------------------------------------------------------------------------
# Define main function
# -----------------------------------------------------------------------------

def main():
    # Deifne the start and end points
    start_pose = RRT.pose_deg(3.0, -5.0, 0)
    final_pose = RRT.pose_deg(0.5, 9.0, -180)

    # Create environment and extract relevant information
    env = carenv.Car(render=False)
    initial_pose, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) # start with reset

    # grow/load the tree
    tree = test_rrt_reverse(obstacles, grow=False, final_pose=final_pose)
    
    start_pose_tree = tree.node_poses[0].copy()
    
    # tree.lookahead = consts.lookahead
    
    tree.print()
    tree.plot()
    plt.show()
    
    plt.rcParams.update({'figure.autolayout': True})
    plt.rcParams.update({
        "font.family": "serif",
        # "font.family": "Helvetica"
    })
    
    
    # fig = plt.figure(1)
    # ax = fig.gca()
    # # ax = tree.plot(ax)
    # tree.map.plot(ax)
    # RRT.plot_pose(ax, start_pose, color="green")
    # RRT.plot_pose(ax, final_pose, color="red")
    # plt.pause(0.1)
    
    plt.figure(2, figsize=(4,3.5))
    times = 5*(0.5+np.arange(len(tree.node_count_per_second)))
    print(f"{sum(tree.node_count_per_second)=}")
    print(f"{sum(tree.attempted_node_count_per_second)=}")
    plt.bar(times, [i/5 for i in tree.attempted_node_count_per_second], width=5.0, label="Rejected nodes", color="darkorange")
    plt.bar(times, [i/5 for i in tree.node_count_per_second], width=5.0, label="Nodes added", color="dodgerblue")
    plt.xlim((0, 100))
    plt.xlabel("Time (s)")
    plt.ylabel("Rate of nodes being added\n(nodes per second)")
    # plt.title("Rate of node addition as tree grows")
    plt.text(100*(1/2), max(tree.attempted_node_count_per_second) * (3.5/5)/5, f"Start pose =\n({start_pose_tree[0]:.1f}m ,{start_pose_tree[1]:.1f}m ,{np.rad2deg(start_pose_tree[2]):.0f}°)", ha="center")
    plt.legend()
    plt.savefig("tree_growth_hard.pdf", bbox_inches='tight')
    # plt.rcParams.update({'font.size': 50})
    # plt.rc('font', size=12)          # controls default text sizes
    # plt.rc('axes', titlesize=12)     # fontsize of the axes title
    # plt.rc('axes', labelsize=8)    # fontsize of the x and y labels
    # plt.rc('xtick', labelsize=8)    # fontsize of the tick labels
    # plt.rc('ytick', labelsize=8)    # fontsize of the tick labels
    # plt.rc('legend', fontsize=20)    # legend fontsize
    # plt.rc('figure', titlesize=20)  # fontsize of the figure title
    
    # plt.figure(7)
    # a = np.array(tree.attempted_node_count_per_second)
    # n = np.array(tree.node_count_per_second)
    # rs = n/a
    # plt.plot(times, (1-rs)*100)
    # plt.ylim((0, 100))
    # plt.ylabel("Rate of node rejection")
    # plt.xlabel("Time (s)")
    
    # plt.figure(8)
    # plt.ylabel("Number of nodes in tree")
    # plt.xlabel("Time (s)")
    # plt.plot(times, np.cumsum(n))
    
    
    # fig, ax1 = plt.subplots()
    # ax2 = ax1.twinx()

    # ax1.plot(times, (1-rs)*100, color="red")
    # ax1.set_ylim((0, 100))
    # ax1.set_ylabel("Rate of node rejection (%)")
    # ax1.set_xlabel("Time (s)")
    # ax2.plot(times, np.cumsum(n)*3, color="green")
    # ax2.set_ylabel("Number of nodes in tree")
    
    # # plt.show()
    
    # plt.figure(3)
    # plt.plot(tree.goal_distance)
    
    
    # final_pose_rev = steer.reverse_pose(final_pose.copy())
    # straight_line_dists = np.linalg.norm(tree.node_poses[:,:2] - final_pose_rev[:2],axis=1)
    # print(f"{straight_line_dists.shape=}")
    # print(f"{tree.node_distances.shape=}")
    
    # ratio = tree.node_distances / straight_line_dists
    
    # plt.figure(4)
    # plt.scatter(ratio, ratio*0, c=range(len(ratio)))
    # plt.hist(ratio, bins=20)
    
    # plt.figure(5)
    # # print(tree.mean_ratios)
    # plt.plot(tree.mean_ratios)
    
    # plt.plot(times, tree.rewire_per_second)
    
    # Run the simulation
    # mujoco_sim(env, start_pose, tree)
    
    plt.show()


def test_rrt_reverse(obstacles, final_pose, grow=False):
    if grow:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose, itera=trange(50000), max_seconds=100)
        with open("tree_hard.pickle", "wb") as outfile:
            # "wb" argument opens the file in binary mode
            pickle.dump(tree, outfile)
    else:
        print("loading...")
        with open("tree_hard.pickle", "rb") as infile:
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
    reroute = False 
    
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
        
        total_error = abs(theta_error) + abs(longitudal_error) + abs(lateral_error)

        ########################## PID ##########################
        steering_angle_theta = theta_pid.pid(theta_error)
        steering_angle_lateral = -lateral_pid.pid(lateral_error)
        throttle = longitudal_pid.pid(longitudal_error)

        ########################## Bounds and combining output of controlllers ##########################
        #combine steering input
        steering_angle = steering_angle_lateral + steering_angle_theta

        #bounds
        if points.shape == None:
            throttle = 0.1
        throttle = bound(-4, 4, throttle)
        steering_angle = bound(-0.38, 0.38, steering_angle)

        ########################## sim ##########################
        action = np.array([steering_angle,-throttle])  #action: first numer [-0.38, 0.38] - = right, + = left. Second number [unconstrained] - backward. + = forwar
        state, obstacles, moving_obstacles = env.step(action) #set step in environment
        env.render(mode = True) # turn rendering on or off
        states = np.vstack((states, state))

        
        ########################## resets when reaching endgoal ##########################

        if np.linalg.norm(state[:2] - points[-1, :2]) < 0.5 and np.linalg.norm(env.get_car_velocity()) < 0.1:
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
            
        if total_error > consts.recompute_error_treshold:
            print(f"error too big ({total_error}, {abs(theta_error)=}, {abs(longitudal_error)=}, {abs(lateral_error)=})")
            points2, reroute = tree.recompute_path(state, obstacles)
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
