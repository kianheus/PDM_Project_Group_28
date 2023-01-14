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
import matplotlib.cm as cm

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


start_pose = RRT.pose_deg(3.0, -5.0, 0)
final_pose = RRT.pose_deg(-6.0, 7.0, 90)

# -----------------------------------------------------------------------------
# Define main function
# -----------------------------------------------------------------------------

def main():
    # Create environment and extract relevant information
    env = carenv.Car(render=False)
    initial_pose, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) # start with reset

    plot(obstacles)
    
    
def test(obstacles):
    env_map = RRT.Map(obstacles, consts=consts)

    # Initialise a RR tree
    plt.figure(1)
    
    tree = RRT.Tree(env_map, initial_pose=start_pose, consts=consts)
    ax = tree.plot()
    RRT.plot_pose(ax, start_pose, color='green')
    RRT.plot_pose(ax, final_pose, color='red')
    plt.pause(0.1)
    
    lengths = []
    times = []
    nodes = []
    colors = iter(cm.rainbow(np.linspace(0, 1, 10)))
    
    paths = []
    
    # Grow the tree
    for i in trange(20):
        
        tree = RRT.Tree(env_map, initial_pose=start_pose, consts=consts)
        tree.final_pose_rev = start_pose
        time_s = time.time()
        tree.grow_to(final_pose, trange(10000), 2*60, finish=True, star=False)
        time_e = time.time()
        times.append(time_e - time_s)
        path = tree.path_to(final_pose)
        lengths.append(path.length)
        nodes.append(len(tree.nodes))
        
        path.plot(ax, color="red", linewidth=1)
        paths.append(path)
        plt.pause(0.1)
        
    print(lengths)
    print(nodes)
    
    lengths = np.array(lengths)
    d = {"lengths": lengths, "paths": paths, "times": times}
    with open("rrt_plain.pickle", "wb") as outfile:
        pickle.dump(d, outfile)
    
    plt.figure(2)
    plt.boxplot(lengths)
    plt.show()


def plot(obstacles):
    env_map = RRT.Map(obstacles, consts=consts)
    with open("rrt_plain.pickle", "rb") as infile:
        d = pickle.load(infile)
    paths_rrt = d["paths"]
    lengths_rrt = d["lengths"]
    times_rrt = d["times"]
    with open("rrt_star.pickle", "rb") as infile:
        d = pickle.load(infile)
    paths_star = d["paths"]
    lengths_star = d["lengths"]
    
    fig = plt.figure(1)
    ax = fig.gca()
    env_map.plot(ax)
    # plt.axes("equal")
    RRT.plot_pose(ax, start_pose, color='green')
    RRT.plot_pose(ax, final_pose, color='red')
    for path in paths_rrt:
        path.plot(ax, color="dodgerblue", linewidth=1)
        path.print()
    for path in paths_star:
        path.plot(ax, color="darkorange", linewidth=1)
        path.print()
    ax.axis("equal")
    # plt.show()
    
    fig = plt.figure(2, figsize=(3.5,3))
    bp = plt.boxplot([lengths_rrt, lengths_star], widths=0.4, patch_artist=True)
    print(bp)
    edge_color, fill_color = "dodgerblue", "white"
    for element in ['whiskers', 'caps']:
        plt.setp(bp[element][:2], color=edge_color, linewidth=1.5)
    for element in ['boxes', 'fliers', 'means', 'medians']:
        plt.setp(bp[element][:1], color=edge_color, linewidth=1.5)
        
    edge_color, fill_color = "darkorange", "white"
    for element in ['whiskers', 'caps']:
        plt.setp(bp[element][2:], color=edge_color, linewidth=1.5)
    for element in ['boxes','fliers', 'means', 'medians']:
        plt.setp(bp[element][1:], color=edge_color, linewidth=1.5)
        
        
    for patch in bp['boxes']:
        patch.set(facecolor=fill_color)    
    plt.xticks(ticks=[1, 2], labels=["RRT","RRT*"])
    # plt.plot([0.5, 1.5], [22.8, 22.8])
    plt.ylim((23, 33))
    plt.ylabel("Distance [m]")
    # plt.boxplot(lengths_star)
    fig.savefig("boxplot.pdf")
    plt.show()
        


def plot_s(obstacles):
    env_map = RRT.Map(obstacles, consts=consts)
    with open("rrt_star.pickle", "rb") as infile:
        d = pickle.load(infile)
    paths_star = d["paths"]
    lengths_star = d["lengths"]
    fig = plt.figure(1)
    ax = fig.gca()
    env_map.plot(ax)
    # plt.axes("equal")
    RRT.plot_pose(ax, start_pose, color='green')
    RRT.plot_pose(ax, final_pose, color='red')
    path :steer.Path = paths_star[1]
    path.plot(ax, color="darkorange", linewidth=1)
    dp = path.interpolate(d=0.1)
    plt.scatter(dp[:,0],dp[:,1])
    path.print()
    ax.axis("equal")
    plt.show()


def test_rrt_reverse(obstacles, final_pose, grow=False):
    if grow:
        tree = RRT.Tree.grow_reverse_tree(obstacles, consts, final_pose=final_pose, itera=trange(10000), max_seconds=10*60)
        with open("tree16.pickle", "wb") as outfile:
            # "wb" argument opens the file in binary mode
            pickle.dump(tree, outfile)
    else:
        print("loading...")
        with open("tree16.pickle", "rb") as infile:
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
