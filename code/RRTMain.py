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


# -----------------------------------------------------------------------------
# Define main fucntion
# -----------------------------------------------------------------------------

def main():
    
    # Create environment and extract relevant information
    env = carenv.Car(render=True)
    state, obstacles, moving_obstacles = env.reset() # start with reset
    obstacles[:,3:] = obstacles[:,3:]*2 # [x, y, rotation, length, width]

    start_time = time.time()
    
    # Define some sets of cooridnates
    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([30, 30]) # Dimensions of workspace
    start_coord = state # Starting position and orientation of robot (x, y, theta)
    goal_coord = np.array([0, 10.05, 0]) # Goal position and orientation of robot (x, y, theta)
    
    # Computational variables
    radius = 0.8
    collision_resolution = 0.05
    
    #test_pygame(start_coord, goal_coord, workspace_size, workspace_center, obstacles)
    points = test_rrt(obstacles, workspace_center, workspace_size, radius, collision_resolution)
    mujoco_sim(env, points)

    #test_rrt_blind(obstacles, workspace_center, workspace_size, radius, collision_resolution)

    #test_approximator(obstacles, workspace_center, workspace_size, radius)


    
# Kian, Thomas
def test_rrt(obstacles, workspace_center, workspace_size, turning_radius, collision_resolution):

    # Set up a environment map object (used for collisions and random point generation)
    env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)

    # Define start and end poses
    initial_pose = RRT.pose_deg(0.5, 0.5, 0)
    final_pose = RRT.pose_deg(9, 5, 0)

    # Initialise a RR tree
    tree = RRT.Tree(env_map, turning_radius=turning_radius, initial_pose=initial_pose, collision_resolution=collision_resolution)
    
    # Grow the tree to the final pose
    done = tree.grow_to(final_pose, trange(1000), 180)
    #tree.rewire(final_pose))

    fig, ax = plt.subplots()
    env_map.plot(ax)    # plot the environment (obstacles)

    # plot the edges of the tree
    for edge in tree.edges:
        edge.path.plot(ax, endpoint=True, color="orange", linewidth=1, alpha=0.3, s=0.4)

    # backtrack the tree to generate a path and a list of points
    points = np.array([[0.0, 0.0, 0.0]])
    if done:
        path = tree.path_to(final_pose)
        path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
        path.print()
        points = path.interpolate_poses(d=0.05)
        plt.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')
        
    # plot the start and endpoints
    steer.plot_point(ax, initial_pose[:2], initial_pose[2], color="green")
    steer.plot_point(ax, final_pose[:2], final_pose[2], color="red")

    ax.set_xlim(-4, 4)
    ax.set_ylim(-4, 4)
    plt.axis("equal")

    plt.show()

    return points


def test_rrt_blind(obstacles, workspace_center, workspace_size, turning_radius, collision_resolution):
    # Set up a environment map object (used for collisions and random point generation)
    env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)

    # Define start and end poses
    initial_pose = RRT.pose_deg(0.0, 0.0, 0)
    final_pose = RRT.pose_deg(2.5, 5.0, 180)

    # Initialise a RR tree
    tree = RRT.Tree(env_map, turning_radius=turning_radius, initial_pose=initial_pose, collision_resolution=collision_resolution)
    
    # Grow the tree
    tree.grow_blind(trange(10000), 1*60)

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

    plt.show()
    """


def test_approximator(obstacles, workspace_center, workspace_size, turning_radius):
    print("Testing approximator")
    DA = Approximator.DubbinsApproximator(turning_radius=turning_radius)

    env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)

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

    state, obstacles, moving_obstacles = env.reset() #start with reset
    starttime = time.time()
    i = 0
    n = 0
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

def local_planner(state, obstacles, moving_obstacles, points, i):
    reroute = False # used for resetting the index i
    offset = 20 # amount of points offsetted from moving obstacle   

    # needed to create map
    workspace_center = np.array([state[0], state[1]]) # Coordinate center of workspace
    #workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([15, 15]) # Dimensions of workspace
    
    # convert to right sizes
    obstacles[:,3:] = obstacles[:,3:]*2 # [x, y, rotation, length, width]
    moving_obstacles[:,3:] = moving_obstacles[:,3:]*2 # [x, y, rotation, length, width]
    
    # only check if path collides with moving obstacles, because normal path is already collision free
    #env_map = RRT.Map(np.vstack((obstacles,moving_obstacles)), 0.1, workspace_center, workspace_size) # checks whole space, noy only workspace
    env_map = RRT.Map(moving_obstacles, 0.1, workspace_center, workspace_size)
    
    # only check all future points for collisions
    points = points[i:]
    collision = env_map.collision_check_array(points)
    index = np.argwhere(collision == True)
    
    # if there are collisions, do local planning
    if index.size != 0: 
        print("Possible collision")
        
        # remove the points that collide, add offset, to see new future goal point
        index = np.arange(np.min(index)-offset, np.max(index)+offset, 1)
        mask = np.ones(points.shape[0], bool)
        mask[index] = False
        points = np.squeeze(points[mask,:])
        start = state
        future_points = points[np.min(index):] # used for creating new path
        #goal = points[np.min(index)]
        goal = future_points[0]
    
        # check if distane between goal and state is within the lookahead distance
        if np.linalg.norm(start - goal) < 7:
            print("collision within range")
            
            # use smaller map to speed up RRT
            workspace_center = np.array([(start[0]+goal[0])/2, (start[1]+goal[1]/2)]) # Coordinate center of workspace
            #workspace_center = np.array([0, 0]) # Coordinate center of workspace
            workspace_size = np.array([start[0]+goal[0]+0.1, 1.6]) # Dimensions of workspace
            
            #workspace_limit = workspace_size/2+workspace_center
            
            # now use all obstacles
            env_map = RRT.Map(np.vstack((obstacles,moving_obstacles)), 0.1, workspace_center, workspace_size) # checks whole space, noy only workspace
            
            #miniRRT(state, obstacles, moving_obstacles, start, goal)
            
            turning_radius = 0.8
            collision_resolution = 0.05
        
            # Initialise a RR tree
            tree = RRT.Tree(env_map, turning_radius=turning_radius, initial_pose=start, collision_resolution=collision_resolution)
        
            # Grow the tree to the final pose
            done = tree.grow_to(goal, trange(200), 0.25)
            
            #fig, ax = plt.subplots()
            #env_map.plot(ax)    # plot the environment (obstacles)   
            #ax.set_xlim(-workspace_size[0]/2 + workspace_center[0], workspace_size[0]/2 + workspace_center[0])
            #ax.set_ylim(-workspace_size[1]/2 + workspace_center[1], workspace_size[1]/2 + workspace_center[1])
            #ax.scatter(goal[0], goal[1])
            
            if done:
                path = tree.path_to(goal)
                #path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
                #path.print()
                updated_points = path.interpolate_poses(d=0.05) # new path to goal
                points = np.vstack([updated_points, future_points]) # combine new and old path
                reroute = True # used to reset index
                #plt.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')
            
            
            #ax.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')
            #ax.axis("equal")
            #plt.show()
            
            #ax.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')
            #ax.scatter(final_path[:,0], final_path[:,1], c=range(final_path.shape[0]), cmap='viridis')
            
        
    else:
        print("follow normal path")
    
    return points, reroute     
    
if __name__ == '__main__':
    main()

