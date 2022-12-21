import carenv
import numpy as np
import time

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
longitudal_pid = PIDcontroller(1, 0, 0.1)

# initialise environment
env = carenv.Car() #load in Car environment

state, obstacles = env.reset() #start with reset

starttime = time.time()

#simulate for 100 s
while True:
    
    ########################## desired goals generated by path planner ########################
    thetar = 30*np.pi/180 # desired heading angle for following path
    pr = np.array([7,1]) #desired point where car needs to go
        
    ########################## heading + longitudal + lateral error ##########################
    #heading
    theta = state[2]
    theta_error = ((thetar-theta+np.pi) % (2*np.pi)) - np.pi #multiple rotations without sudden gap between 0 and 2pi   
    
    print(theta_error)
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
    steering_angle = steering_angle_theta + steering_angle_lateral
    
    #bounds
    throttle = bound(-1, 1, throttle)
    steering_angle = bound(-0.38, 0.38, steering_angle)
    
    ########################## sim ##########################
    action = np.array([steering_angle,-throttle])  #action: first numer [-0.38, 0.38] - = right, + = left. Second number [unconstrained] - backward. + = forwar
    state, obstacles = env.step(action) #set step in environment
    env.render(mode = True) # turn rendering on or off

    ########################## reset every 20 seconds, this can be changed ##########################   
    if env.get_time() > 20:
        env.reset()
    
    time.sleep(0.01 - ((time.time() - starttime) % 0.01)) # sleep for 100 Hz realtime loop
        