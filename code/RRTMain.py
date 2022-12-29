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
    env = carenv.Car(render=True)
    state, obstacles = env.reset() #start with reset
    obstacles[:,3:] = obstacles[:,3:]*2 # [x, y, rotation, length, width]

    start_time = time.time()
    
    # Define some sets of cooridnates
    workspace_center = np.array([0, 0]) # Coordinate center of workspace
    workspace_size = np.array([30, 30]) # Dimensions of workspace
    start_coord = state # Starting position and orientation of robot (x, y, theta)
    goal_coord = np.array([0, 10.05, 0]) # Goal position and orientation of robot (x, y, theta)
    
    #Computational variables
    radius = 1.2
    collision_resolution = 0.05
    
    # test_pygame(start_coord, goal_coord, workspace_size, workspace_center, obstacles)
    points = test_rrt(obstacles, workspace_center, workspace_size, radius, collision_resolution)
    mujoco_sim(env, points)

    
# Kian, Thomas
def test_rrt(obstacles, workspace_center, workspace_size, turning_radius, collision_resolution):
    env_map = RRT.Map(obstacles, 0.1, workspace_center, workspace_size)

    initial_pose = RRT.pose_deg(0.0, 0.0, 0)
    final_pose = RRT.pose_deg(5.0, 3.0, -90)

    tree = RRT.Tree(env_map, turning_radius=turning_radius, initial_pose=initial_pose, collision_resolution=collision_resolution)
    
    done = tree.grow_to(final_pose, trange(2000), 180)

    fig, ax = plt.subplots()
    env_map.plot(ax)

    for edge in tree.edges:
        # print("edge")
        edge.path.plot(ax, endpoint=True, color="orange", linewidth=1, alpha=0.3, s=0.4)


    points = np.array([[0.0, 0.0, 0.0]])
    if done:
        path = tree.path_to(final_pose)
        path.plot(ax, endpoint=True, color="red", linewidth=3, alpha=1.0, s=1.0)
        path.print()
        points = path.interpolate_angles_2(d=0.05)
        plt.scatter(points[:,0], points[:,1], c=range(points.shape[0]), cmap='viridis')
        

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



# Fabio                
def mujoco_sim(env, points):
    
    points = np.flipud(points)
    
    
    #function used to bound output of controllers
    def bound(low, high, value):
         return max(low, min(high, value))

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

    state, obstacles = env.reset() #start with reset
    print(obstacles)
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
        state, obstacles = env.step(action) #set step in environment
        env.render(mode = True) # turn rendering on or off

        ########################## reset after 10 seconds, this can be changed ##########################   
        if env.get_time() > 20:
            env.close_window()
            break
        
        if n % 2 == 0:
            i = i + 1
            i = bound(0, points.shape[0]-1, i)
        
        n = n+1
        time.sleep(0.01 - ((time.time() - starttime) % 0.01)) # sleep for 100 Hz realtime loop



if __name__ == '__main__':
    main()

