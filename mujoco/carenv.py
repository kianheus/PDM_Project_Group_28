from gym import core
import mujoco
import mujoco_viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

class Car(core.Env):
                      
    def __init__(self):
        
        #open model
        self.model = mujoco.MjModel.from_xml_path('models/hospital.xml')
        self.data = mujoco.MjData(self.model)
        
        #open viewer
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width = 1000, height = 1000, hide_menus = True)
      
    def reset(self):
        
        #reset mujoco model
        mujoco.mj_resetCallbacks()
        mujoco.mj_resetData(self.model, self.data)
        mujoco.mj_step(self.model, self.data) #set one step of model to be sure that everything is initialised
        
        #extract car and world data
        data = self.get_sensor_data()    
        obstacles = self.get_obstacles_simple()        
        state = {'x' : data['car_pos'][0], 'y' : data['car_pos'][1], 'theta' : data['car_orientation']}
        
        return state, obstacles

    def step(self, action):
        self.data.ctrl[0] = action[0]
        self.data.ctrl[1] = action[1]
             
        mujoco.mj_step(self.model, self.data)
        
        data = self.get_sensor_data()
        
        obstacles = self.get_obstacles_simple()
        
        state = {'x' : data['car_pos'][0], 'y' : data['car_pos'][1], 'theta' : data['car_orientation']}
        
        return state, obstacles
    
    def render(self, mode):
        self.viewer.render()
        
    def get_sensor_data(self):
        
        rl = R.from_quat(self.data.sensor('buddy_quat').data)
        orientation = - (rl.as_euler('xyz')[0] - np.pi)
            
        data = {'car_pos': self.data.sensor('buddy_pos').data[0:3],
                'car_orientation' : orientation}
        
        return data
    
    def quat_to_degree(self, quat):
        rl = R.from_quat(quat)
        orientation = - (rl.as_euler('xyz')[0] - np.pi)
        return orientation
        
    def get_obstacles_simple(self):
        
        obs = np.array([np.hstack(([self.data.body('wall1').xpos[0:2], self.quat_to_degree(self.data.body('wall1').xquat), self.model.geom('wall1').size[0:2]])),
                        np.hstack(([self.data.body('wall2').xpos[0:2], self.quat_to_degree(self.data.body('wall2').xquat), self.model.geom('wall2').size[0:2]])),
                        np.hstack(([self.data.body('wall3').xpos[0:2], self.quat_to_degree(self.data.body('wall3').xquat), self.model.geom('wall3').size[0:2]])),
                        np.hstack(([self.data.body('wall4').xpos[0:2], self.quat_to_degree(self.data.body('wall4').xquat), self.model.geom('wall4').size[0:2]])),
                        np.hstack(([self.data.body('wallhall1').xpos[0:2], self.quat_to_degree(self.data.body('wallhall1').xquat), self.model.geom('wallhall1').size[0:2]])),
                        np.hstack(([self.data.body('wallhall2').xpos[0:2], self.quat_to_degree(self.data.body('wallhall2').xquat), self.model.geom('wallhall2').size[0:2]])),
                        np.hstack(([self.data.body('wallhall3').xpos[0:2], self.quat_to_degree(self.data.body('wallhall3').xquat), self.model.geom('wallhall3').size[0:2]])),
                        np.hstack(([self.data.body('wallhall4').xpos[0:2], self.quat_to_degree(self.data.body('wallhall4').xquat), self.model.geom('wallhall4').size[0:2]])),
                        np.hstack(([self.data.body('wallhall5').xpos[0:2], self.quat_to_degree(self.data.body('wallhall5').xquat), self.model.geom('wallhall5').size[0:2]])),
                        np.hstack(([self.data.body('wallhall6').xpos[0:2], self.quat_to_degree(self.data.body('wallhall6').xquat), self.model.geom('wallhall6').size[0:2]])),
                        np.hstack(([self.data.body('wallhall7').xpos[0:2], self.quat_to_degree(self.data.body('wallhall7').xquat), self.model.geom('wallhall7').size[0:2]])),
                        np.hstack(([self.data.body('wallhall8').xpos[0:2], self.quat_to_degree(self.data.body('wallhall8').xquat), self.model.geom('wallhall8').size[0:2]]))])

        return obs
    
    def get_time(self):
        time = self.data.time
        return time
    
    def close_window(self):
        self.viewer.close()
    
    """
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
    """    