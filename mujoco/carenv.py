from gym import core
import mujoco
import mujoco_viewer
import numpy as np
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------------
# Custom environment class using the mujoco physics simulation and the mujoco_viewer for visualisation
# -----------------------------------------------------------------------------

class Car(core.Env):
    
    # initialisation of environment
    def __init__(self, render = True):
        
        # load mujoco models and intialise the data
        self.model = mujoco.MjModel.from_xml_path('../mujoco/models/hospital.xml')
        self.data = mujoco.MjData(self.model)
        
        # set constants
        self.counter = 0
        self.bedspeed = 1
        
        # initiase visualisation window
        if render == True:   
            self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data, width = 1000, height = 1000, hide_menus = True)

    # reset mujoco environment
    def reset(self, x, y, theta):
        
        # reset mujoco model and data
        mujoco.mj_resetCallbacks()
        mujoco.mj_resetData(self.model, self.data)
        
        # put car into new position
        self.set_position(x, y, theta)
        
        #set one step of model to be sure that everything is initialised
        mujoco.mj_step(self.model, self.data) 
        
        # extract data of all sensors, obstacles and moving obstacles 
        data = self.get_sensor_data()
        state = np.array([data['car_pos'][0], data['car_pos'][1], data['car_orientation']])
        obstacles = self.get_obstacles()
        moving_obstacles = self.get_moving_obstacle()
        moving_obstacles_speed = self.get_moving_obstacle_speed()
        
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
        
        # simulate 0.01 seconds of the simulation
        mujoco.mj_step(self.model, self.data)
        
        # extract data of all sensors, obstacles and moving obstacles 
        data = self.get_sensor_data()
        state = np.array([data['car_pos'][0], data['car_pos'][1], data['car_orientation']])
        obstacles = self.get_obstacles()
        moving_obstacles = self.get_moving_obstacle()
        moving_obstacles_speed = self.get_moving_obstacle_speed()
        
        # counter used for moving bed
        self.counter = self.counter + 1

        return state, obstacles, moving_obstacles
    
    # render a frame
    def render(self, mode):
        self.viewer.render()
    
    # set the desired location for the vehicle
    def set_position(self, x, y, theta):
        self.data.qpos = [x,y,0,np.cos(theta/2),0,0,np.sin(theta/2),0,0,0,0,0,0,0,0] # x,y,z, queternion [0, 1, 2, 3], ???
   
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
        obs = np.array([np.hstack(([self.data.body('movingbed1').xpos[0:2], self.quat_to_degree(self.data.body('movingbed1').xquat), np.array([1,0.5])]))])
        obs[:,3:] = obs[:,3:]*2      
        return obs
    
    # moving obstacle velocity, in x, y, z direction
    def get_moving_obstacle_speed(self):
        obs_vel = np.array([self.data.body('movingbed1').cvel[3:6]])
        return obs_vel
    
    # returns simulation time
    def get_time(self):
        time = self.data.time
        return time
    
    # closing the mujoco viewer window
    def close_window(self):
        self.viewer.close()
 