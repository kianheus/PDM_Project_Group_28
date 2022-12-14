import carenv
import numpy as np
import time

class PIDcontroller():
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.pe = 0
        self.se = 0
        self.dt = 0.01
        
    def pid(self, e):
       
        self.se = self.se + e*self.dt # intergrated error
         
        P = self.kp*e 
        I = self.ki*self.se #Forward euler
        D = self.kd*(e-self.pe)/dt #Forward difference 
        
        output = P + I + D # Output of pid controller
        
        self.pe = e # previous error used in differentiation
        
        return output

theta_pid = PIDcontroller(1, 0, 0)
longitudal_pid = PIDcontroller(1, 0, 0)
        
env = carenv.Car() #load in Car environment

state, obstacles = env.reset() #start with reset

starttime = time.time()

pe = 0
se = 0
Kp = 1
Ki = 0
Kd = 0
dt = 0.01
thetar = 0*2*np.pi
pr = np.array([0,0])


#simulate for 100 s
while True:
    
    #action: first numer [-0.38, 0.38] - = right, + = left. Second number [unconstrained] - backward. + = forwar
    
    ########################## angle    
    theta = state['theta']
    theta_error = ((thetar-theta+np.pi) % (2*np.pi)) - np.pi # error
    steering_angle = theta_pid.pid(theta_error)
    
    ########################## lateral error
    p = np.array([state['x'], state['y']])
    
    distance = np.linalg.norm(pr-p)
    
    angle = np.arctan2(p[1],p[0])
    
    difference_angle = thetar - angle
    
    lateral_error = -np.sin(difference_angle)*distance
    longitudal_error = np.cos(difference_angle)*distance
     
    throttle = longitudal_pid.pid(longitudal_error)
    
    action = np.array([steering_angle,throttle]) 
    state, obstacles = env.step(action) #set step in environment
    env.render(mode = True) # turn rendering on or off

    #reset every 5 seconds, this can be changed    
    if env.get_time() > 20:
        env.reset()
    
    #print(state)        
    time.sleep(0.01 - ((time.time() - starttime) % 0.01))
        