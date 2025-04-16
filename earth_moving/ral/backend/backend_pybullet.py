import pybullet as p
import pybullet_data
import time as t
import os
import matplotlib.pyplot as plt
import numpy as np

from earth_moving.ral.backend.backend_base import BackendBase
from earth_moving.ral.sensor.sensor_backend import BaseSensorBackend
from earth_moving.ral.robot.robot_backend import BaseRobotBackend
from earth_moving.ral.algorithms.probabilistic_simulator import ProbabilisticSimulator


class BackendPybullet(BackendBase):
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        self._timedelta = kwargs.get('timedelta')
        self._update_period = kwargs.get('update_period')
        self._gui = kwargs.get('gui')  
        if self._gui:
            self._physicsClient = p.connect(p.GUI)
        else:
            self._physicsClient = p.connect(p.DIRECT)
        self._gravity = kwargs.get('gravity')
        p.setGravity(self._gravity[0], self._gravity[1], self._gravity[2])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  #  PyBullet_data package (see doc)       
        self._ID = []        
        self._ID.append(p.loadURDF("plane.urdf"))        
        
        self._ID_aggregates = []
        self._current_time = 0
        
    def step(self):
        p.stepSimulation()        
        self._current_time += self._timedelta
        t.sleep(self._timedelta)
        
    def spawn_object(self, object_pose, object_description):        
        position = object_pose[0:2]
        orientation = object_pose[3:-1]                        
        self._ID.append(p.loadURDF(object_description, basePosition=position, baseOrientation=orientation))
        self._ID_aggregates.append(self._ID[-1])       
            
    def initiate_rgb_sensor(self,**kwargs) -> BaseSensorBackend: # TODO: unlike ROS, this needs to happen for all sensors that we want at the beginning of the run
        
        class PybulletSensorRGBBackend(BaseSensorBackend):
            
            def __init__(self,**kwargs) -> None:
                super().__init__(**kwargs)
                super().initiate_sensor(**kwargs)
                
                self._imgW = self._sensor.get('imgW')
                self._imgH = self._sensor.get('imgH')
                
                self._camera_target_pose = self._sensor.get('camera_target_pose')
                self._camera_distance = self._sensor.get('camera_distance')
                self._yaw = self._sensor.get('yaw')
                self._pitch = self._sensor.get('pitch')
                self._roll = self._sensor.get('roll')
                self._up_axis_index = self._sensor.get('up_axis_index')
                self._viewMatrix = p.computeViewMatrixFromYawPitchRoll(self._camera_target_pose, 
                                                                       self._camera_distance, 
                                                                       self._yaw, 
                                                                       self._pitch, 
                                                                       self._roll, 
                                                                       self._up_axis_index)
                
                self._fov = self._sensor.get('fov')
                self._aspect_ratio = self._sensor.get('aspect_ratio')                
                self._near = self._sensor.get('near')
                self._far = self._sensor.get('far')
                self._projectionMatrix = p.computeProjectionMatrixFOV(self._fov, 
                                                                      self._aspect_ratio, 
                                                                      self._near, 
                                                                      self._far)
                
                self._save_path = self._sensor.get('save_path')
            
            def get_data(self) -> np.array:                
                data = p.getCameraImage(self._imgW,self._imgH,self._viewMatrix,self._projectionMatrix,renderer=p.ER_BULLET_HARDWARE_OPENGL)
                return data
            
            def plot_data(self,**kwargs):
                data = kwargs.get('data')
                rgb = np.reshape(data[2], (self._imgH, self._imgW, 4)) * 1. / 255.
                plt.imshow(rgb)
                plt.title('RGB image')
                plt.show()
                
            def save_data(self,**kwargs):
                data = kwargs.get('data')
                save_path = kwargs.get('path')
                rgb = np.reshape(data[2], (self._imgH, self._imgW, 4)) * 1. / 255.
                name =  kwargs.get('name') + '.png'
                if name in os.listdir(save_path):
                    os.remove(save_path + name)
                plt.title('RGB image')
                plt.imsave(save_path + name,rgb,format='png')
                
        sensor_backend = PybulletSensorRGBBackend(**kwargs)
        return sensor_backend