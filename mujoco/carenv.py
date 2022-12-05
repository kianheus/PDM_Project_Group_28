from gym import core
import mujoco
import mujoco_viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

class Car(core.Env):
                      
    def __init__(self):
        
        self.model = mujoco.MjModel.from_xml_path('models/one_car.xml')
        
        self.data = mujoco.MjData(self.model)
        
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width = 1000, height = 1000, hide_menus = True)
    
        
    def reset(self):
        
        mujoco.mj_resetCallbacks()
        mujoco.mj_resetData(self.model, self.data)
           
        for i in range(1):
           mujoco.mj_step(self.model, self.data)

        obs=0
        return obs

    def step(self, action):
        self.data.ctrl[0] = action[0]
        self.data.ctrl[1] = action[1]
             
        mujoco.mj_step(self.model, self.data)
        
        reward = 0
        done = 0
        info = 0
        
        data = self.get_sensor_data()
        
        obs = {'x' : data['car_pos'][0], 'y' : data['car_pos'][1], 'theta' : data['car_orientation']}
        
        return obs, reward, done, info
    
    def render(self, mode):
        self.viewer.render()
        
    def get_sensor_data(self):
        
        rl = R.from_quat(self.data.sensor('buddy_quat').data)
        orientation = - (rl.as_euler('xyz')[0] - np.pi)
            
        data = {'car_pos': self.data.sensor('buddy_pos').data[0:3],
                'car_orientation' : orientation}
        
        return data
    
    def get_time(self):
        time = self.data.time
        return time
    
    def close_window(self):
        self.viewer.close()
        