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

import Approximator



# np.random.seed(28)


class consts():
    turning_radius = 0.8
    collision_resolution = 0.05
    point_resolution = 0.05
    vehicle_radius = 0.2
    workspace_center = np.array([0, 0])
    workspace_size = np.array([30, 30])

#Define start pose
start_pose = RRT.pose_deg(-1.5, 5, 180)

# -----------------------------------------------------------------------------
# Define main fucntion
# -----------------------------------------------------------------------------

def main():
    # Create environment and extract relevant information
    env = carenv.Car(render=False)
    initial_pose, obstacles, moving_obstacles = env.reset(start_pose[0], start_pose[1], start_pose[2]) # start with reset
    # [x, y, rotation, length, width]

    #test_pygame(start_coord, goal_coord, workspace_size, workspace_center, obstacles)
    points, ax = test_rrt(obstacles, initial_pose)
    env = carenv.Car(render=True)
    mujoco_sim(env, points)

    # test_rrt_blind(obstacles)

    #test_approximator(obstacles)

# Kian, Thomas
def test_rrt(obstacles, initial_pose=RRT.pose_deg(0, 0, 0), plot=True):

    # Set up a environment map object (used for collisions and random point generation)
    env_map = RRT.Map(obstacles, consts=consts)
    
    # Define end poses
    final_pose = RRT.pose_deg(5, 5, 180)

    # Initialise a RR tree
    tree = RRT.Tree(env_map, initial_pose=initial_pose, consts=consts)
    
    # Grow the tree to the final pose
    done = tree.grow_to(final_pose, trange(1000), 180)
    #tree.rewire(final_pose))

    if plot:
        fig, ax = plt.subplots()
        env_map.plot(ax)    # plot the environment (obstacles)

        # plot the edges of the tree
        for edge in tree.edges:
            edge.path.plot(ax, endpoint=True, color="orange", linewidth=1, alpha=0.3, s=0.4)

    # backtrack the tree to generate a path and a list of points
    points = np.array([[0.0, 0.0, 0.0]])
    if done:

        path = tree.path_to(final_pose)
        path.print()
        points = path.interpolate_poses(d=consts.point_resolution)
        if plot:
            path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
            RRT.plot_points(ax, points, cmap='viridis')
        
    if plot:
        # plot the start and endpoints
        RRT.plot_pose(ax, initial_pose, color="green")
        RRT.plot_pose(ax, final_pose, color="red")

        ax.set_xlim(-4, 4)
        ax.set_ylim(-4, 4)
        plt.axis("equal")

        plt.pause(0.1)

    return points, ax


def test_rrt_blind(obstacles):
    # Set up a environment map object (used for collisions and random point generation)
    env_map = RRT.Map(obstacles, consts=consts)

    # Define start and end poses
    initial_pose = RRT.pose_deg(0.0, 0.0, 0)

    final_pose = RRT.pose_deg(1.5, 5.0, 180)

    # Initialise a RR tree
    tree = RRT.Tree(env_map, initial_pose=initial_pose, consts=consts)
    
    # Grow the tree
    tree.grow_blind(trange(10000), 1*60)
    
    tree.add_path_to(final_pose, modify_angle=False)
    print(tree.get_node(final_pose))
  
    path = tree.path_to(final_pose)
    
    tree.print()
    
    """
    fig, ax = plt.subplots()
    env_map.plot(ax)    # plot the environment (obstacles)

    # plot the edges of the tree
    for edge in tree.edges:
        edge.path.plot(ax, endpoint=True, color="orange", linewidth=1, alpha=0.3, s=0.4)

    # plot the start and endpoints
    steer.plot_point(ax, initial_pose[:2], initial_pose[2], color="green")
    steer.plot_point(ax, final_pose[:2], final_pose[2], color="red")

    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    plt.axis("equal")
    
    path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
    path.print()
    points = path.interpolate_poses(d=0.05)
    plt.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')

    plt.show()
    """


def test_approximator(obstacles):
    print("Testing approximator")
    DA = Approximator.DubbinsApproximator(turning_radius=consts.turning_radius)

    env_map = RRT.Map(obstacles, consts=consts)

    start_pose = env_map.random_pose()
    end_pose = env_map.random_pose()

    # Options for the interpolation are:
    # Approximator.InterpolationType.Interpolated (interpolates value, should be pretty good most of the time)
    # Approximator.InterpolationType.Nearest (looks up nearest value, should be a bit worse than interpolate, but might handle discontinuities better)
    # Approximator.InterpolationType.UpperBound (should always be bigger or equal to the true value, not implemented yet)
    dist_approximated = DA.lookup(start_pose, end_pose, Approximator.InterpolationType.Nearest)
    dist_true = steer.optimal_path(start_pose, end_pose, turning_radius).length

    print(f"{dist_true=}")
    print(f"{dist_approximated=}")




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




# Fabio    

def bound(low, high, value):
     return max(low, min(high, value))
            
def mujoco_sim(env, points):
    #function used to bound output of controllers
    

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

    original_points = points.copy()

    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([30, 30]) # Dimensions of workspace
    env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)
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

        
        if n % 4 == 0:
            i = i + 1
            i = bound(0, points.shape[0]-1, i)
            
        if n % 10 == 0:
            points2, reroute = local_planner(state, obstacles, moving_obstacles, points, i)
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

def local_planner(state, obstacles, moving_obstacles, points, i):
    reroute = False # used for resetting the index i
    offset = 20 # amount of points offsetted from moving obstacle   

    # needed to create map
    workspace_center = np.array([state[0], state[1]]) # Coordinate center of workspace
    #workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([15, 15]) # Dimensions of workspace
    
    # only check if path collides with moving obstacles, because normal path is already collision free
    #env_map = RRT.Map(np.vstack((obstacles,moving_obstacles)), 0.1, workspace_center, workspace_size) # checks whole space, noy only workspace
    env_map = RRT.Map(moving_obstacles, workspace_center=workspace_center, workspace_size=workspace_size, vehicle_radius=consts.vehicle_radius)
    
    # only check all future points for collisions
    prev_points = points.copy()
    points = points[i:]
    collision = env_map.collision_check_array(points)
    index = np.argwhere(collision == True)
    
    # if there are collisions, do local planning
    if index.size != 0: 
        print("Possible collision")
        # remove the points that collide, add offset, to see new future goal point
        #print(index)
        index = np.arange(np.min(index)-offset, np.max(index)+offset, 1)
        first_coliding_point = points[np.min(index)]
        #print(index)
        mask = np.ones(points.shape[0], bool)
        mask[index] = False
        coliding_points = points[~mask,:]
        points = np.squeeze(points[mask,:])
        start = state
        future_points = points[np.min(index):] # used for creating new path

        before_points = points[:np.min(index)]
        coliding_points = np.vstack((before_points, coliding_points)) # these points are being removed
        # goal = points[np.max(index)]
        try:
            goal = future_points[0]
        except:
            print(f"{future_points=}")
            print(f"{np.max(index)=}")
            print(f"{np.min(index)=}")
            exit()

        # check if distane between goal and state is within the lookahead distance
        if np.linalg.norm(start[:2] - first_coliding_point[:2]) < 3:
            print("collision within range")
            
            # use smaller map to speed up RRT
            workspace_center = np.array([(start[0]+goal[0])/2, (start[1]+goal[1]/2)]) # Coordinate center of workspace
            workspace_size = np.array([max(start[0]+goal[0]+0.1, 1.6), max(start[1]+goal[1], 1.6)]) # Dimensions of workspace

            # now use all obstacles
            env_map = RRT.Map(np.vstack((obstacles,moving_obstacles)), workspace_center=workspace_center, workspace_size=workspace_size, vehicle_radius=consts.vehicle_radius) # checks whole space, noy only workspace
        
            # Initialise a RR tree
            tree = RRT.Tree(env_map, initial_pose=start, consts=consts)
        
            # Grow the tree to the final pose
            done = tree.grow_to(goal, trange(200), 0.25, star=False)
            
            # fig, ax = plt.subplots()
            # env_map.plot(ax)    # plot the environment (obstacles)   
            # ax.set_xlim(-workspace_size[0]/2 + workspace_center[0], workspace_size[0]/2 + workspace_center[0])
            # ax.set_ylim(-workspace_size[1]/2 + workspace_center[1], workspace_size[1]/2 + workspace_center[1])
            # ax.scatter(goal[0], goal[1])
            
            if done:
                path = tree.path_to(goal)
                #path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
                #path.print()
                if path is not None:
                    updated_points = path.interpolate_poses(d=0.05) # new path to goal
                    points = np.vstack([updated_points, future_points]) # combine new and old path
                    reroute = True # used to reset index
                else:
                    print(f"Could not find path, even though there should be one!!!")

                # ax.scatter(updated_points[:,0], updated_points[:,1], c=range(updated_points.shape[0]), cmap='summer')

            
            
            # ax.scatter(future_points[:,0], future_points[:,1], c=range(future_points.shape[0]), cmap='winter')
            # ax.scatter(coliding_points[:,0], coliding_points[:,1], c=range(coliding_points.shape[0]), cmap='autumn')
            # RRT.plot_pose(ax, start, color='green')
            # RRT.plot_pose(ax, goal, color='blue')
            # RRT.plot_pose(ax, first_coliding_point, color='red')
            # ax.axis("equal")
            # plt.show()
    else:
        print("follow normal path")
    
    return points, reroute     
    
if __name__ == '__main__':
    main()
