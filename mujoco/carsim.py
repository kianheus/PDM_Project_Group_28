import carenv
import numpy as np
import time

env = carenv.Car() #load in Car environment

env.reset() #start with reset

starttime = time.time()

#simulate for 100 s
while True:
    
    
    #action: first numer [-0.38, 0.38] - = right, + = left. Second number [unconstrained] - backward. + = forward
    action = np.array([0,0.1]) 
    state, reward, done, info = env.step(action) #set step in environment
    env.render(mode = True) # turn rendering on or off

    #reset every 5 seconds, this can be changed    
    if env.get_time() > 50:
        env.reset()
    
    #print(state)        
    time.sleep(0.01 - ((time.time() - starttime) % 0.01))
        