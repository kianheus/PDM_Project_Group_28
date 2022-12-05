from gym import core
import mujoco
import mujoco_viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

class Car(core.Env):
                      
    def __init__(self):
        
        self.model = mujoco.MjModel.from_xml_path('models/hospital.xml')
        
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
        obs = 0
        
        data = self.get_sensor_data()
        
        self.get_obstacles()
        
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
    
    def get_obstacles(self):
        
        def view1D(a, b): # a, b are arrays
            a = np.ascontiguousarray(a)
            b = np.ascontiguousarray(b)
            void_dt = np.dtype((np.void, a.dtype.itemsize * a.shape[1]))
            return a.view(void_dt).ravel(),  b.view(void_dt).ravel()

        def setdiff_nd(a,b):
            # a,b are the nD input arrays
            A,B = view1D(a,b)    
            return a[~np.isin(A,B)]
        
        
        notobstpos = {k: self.data.body(k).xpos for k in ('buddy', 'buddy_steering_wheel', 'buddy_wheel_bl', 'buddy_wheel_br', 'buddy_wheel_fl', 'buddy_wheel_fr', 'world')}
        #notobstsize = {k: self.model.body(k).size for k in ('buddy', 'buddy_steering_wheel', 'buddy_wheel_bl', 'buddy_wheel_br', 'buddy_wheel_fl', 'buddy_wheel_fr', 'world')}
        
        result = notobstpos.items()
        data = list(result)
        notobstpos = np.array(data)
        
        #result = notobstsize.items()
        #data = list(result)
        #notobstsize = np.array(data)
        
        print(self.data.geom_xpos)
        
        obstpos = setdiff_nd(self.data.geom_xpos, notobstpos)
        
        print(obstpos)
        
        #print(self.data.body('buddy').xpos)
        #print('pos = ', self.data.geom_xpos[1:ngeom+1])
        #print('size = ',self.model.geom_size[1:ngeom+1])
        #print(self.data.geom('wall1').xpos)
        #print(self.model.geom('wall1').size)
    
    def get_time(self):
        time = self.data.time
        return time
    
    def close_window(self):
        self.viewer.close()
        