from gym import core
import mujoco
import mujoco_viewer
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image

# -----------------------------------------------------------------------------
# Custom environment class using the mujoco physics simulation and the mujoco_viewer for visualisation
# -----------------------------------------------------------------------------

class Car(core.Env):
    
    # initialisation of environment
    def __init__(self, mode):
        
        # load mujoco models and intialise the data
        self.model = mujoco.MjModel.from_xml_path('../mujoco/models/hospital.xml')
        self.data = mujoco.MjData(self.model)
        
        # set constants
        self.counter = 0
        self.bedspeed = 1
        
        self.mode = mode
        
        # initiase visualisation window for live rendering
        if self.mode == 1:   
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width = 1000, height = 1000, hide_menus = True)
        
        # initiase visualisation window for offscreen rendering
        elif self.mode == 2: 
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, 'offscreen')

    # reset mujoco environment
    def reset(self, loc):
        
        # reset mujoco model and data
        mujoco.mj_resetCallbacks()
        mujoco.mj_resetData(self.model, self.data)
        
        # put car into new position
        self.set_position(loc)
        
        #set one step of model to be sure that everything is initialised
        mujoco.mj_step(self.model, self.data) 
        
        # extract data of all sensors, obstacles and moving obstacles 
        data = self.get_sensor_data()
        state = np.array([data['car_pos'][0], data['car_pos'][1], data['car_orientation']])
        obstacles = self.get_obstacles()
        moving_obstacles = self.get_moving_obstacle()
        
        # reset counter and bedspeed for moving obstacles
        self.counter = 0   
        self.bedspeed = 1

        return state, obstacles, moving_obstacles
    
    # one simulation step of 0.01 second on the simulation environment
    def step(self, action):
        
        # car steering[0] and throttle[1]
        self.data.ctrl[0] = action[0]
        self.data.ctrl[1] = action[1]
        
        # used for controlling the moving bed that is put on a slider. After 2000 simulation steps the velocity switches signs
        if self.counter % 2000 == 0:
            self.bedspeed = self.bedspeed*-1
        self.data.ctrl[2] = self.bedspeed
        self.data.ctrl[3] = -self.bedspeed
        
        # simulate 0.01 seconds of the simulation
        mujoco.mj_step(self.model, self.data)
        
        # extract data of all sensors, obstacles and moving obstacles 
        data = self.get_sensor_data()
        state = np.array([data['car_pos'][0], data['car_pos'][1], data['car_orientation']])
        obstacles = self.get_obstacles()
        moving_obstacles = self.get_moving_obstacle()
        
        # counter used for moving bed
        self.counter = self.counter + 1

        return state, obstacles, moving_obstacles
    
    # render a frame
    def render(self):
        # live render
        if self.mode == 1:
            self.viewer.render()
            
        # offscreen render
        elif self.mode ==2:
            img = self.viewer.read_pixels(camid=1)
            im = Image.fromarray(img)
            im.save("../videoframes/frame_{}.jpeg".format(self.counter))
        
    # render a frame
    def off_screen_render(self, mode):
        self.viewer.render()
        
    # set the desired location for the vehicle
    def set_position(self, loc):
        self.data.qpos = [loc[0],loc[1],0,np.cos(loc[2]/2),0,0,np.sin(loc[2]/2),0,0,0,0,0,0,0,0,0] # x,y,z, queternion [0, 1, 2, 3], ???
   
    # returns the state of the vehicle via mujoco sensors. x,y,z postion and orientation around the z-axis
    def get_sensor_data(self):

        rl = R.from_quat(self.data.sensor('buddy_quat').data)
        orientation = - (rl.as_euler('xyz')[0] - np.pi)

        data = {'car_pos': self.data.sensor('buddy_pos').data[0:3],
                'car_orientation' : orientation}

        return data
    
    # converts queterions to the euler angle around the z axis
    def quat_to_degree(self, quat):
        rl = R.from_quat(quat)
        orientation = - (rl.as_euler('xyz')[0] - np.pi)
        return orientation
    
    # not moving obstacles. x,y postion, orientation around z-axis, "x,y" size
    def get_obstacles(self):
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
                        np.hstack(([self.data.body('wallhall8').xpos[0:2], self.quat_to_degree(self.data.body('wallhall8').xquat), self.model.geom('wallhall8').size[0:2]])),
                        np.hstack(([self.data.body('hospitalbed1').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed1').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed2').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed2').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed3').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed3').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed4').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed4').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed5').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed5').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed6').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed6').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed7').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed7').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed8').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed8').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed9').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed9').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed10').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed10').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed11').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed11').xquat), np.array([1.1,0.55])])),
                        np.hstack(([self.data.body('hospitalbed12').xpos[0:2], self.quat_to_degree(self.data.body('hospitalbed12').xquat), np.array([1.1,0.55])]))])
        obs[:,3:] = obs[:,3:]*2         
        return obs
    
    # moving obstacles. x,y postion, orientation around z-axis, "x,y" size
    def get_moving_obstacle(self): 
        obs = np.array([np.hstack(([self.data.body('movingbed1').xpos[0:2], self.quat_to_degree(self.data.body('movingbed1').xquat), np.array([1,0.5])])),
                        np.hstack(([self.data.body('movingbed2').xpos[0:2], self.quat_to_degree(self.data.body('movingbed2').xquat), np.array([1,0.5])]))])
        obs[:,3:] = obs[:,3:]*2.0   
        obs[:,3] = obs[:,3]*1.25  
        return obs
    
    # car obstacle velocity, in x, y direction
    def get_car_velocity(self):
        vel = np.array([self.data.body('buddy').cvel[3:5]])
        return vel
    
    # returns simulation time
    def get_time(self):
        time = self.data.time
        return time
    
    # closing the mujoco viewer window
    def close_window(self):
        self.viewer.close()
 