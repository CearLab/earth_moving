import pybullet as p
import pybullet_data
import time as t
import os
import matplotlib.pyplot as plt
import numpy as np

from ral.backend.base_backend import BaseBackend
from ral.sensor.sensor_backend import BaseSensorBackend

class PybulletBackend(BaseBackend):
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._gui = simulation.get('gui')  
        if self._gui:
            self._physicsClient = p.connect(p.GUI)
        else:
            self._physicsClient = p.connect(p.DIRECT)
        self._gravity = simulation.get('gravity')
        p.setGravity(self._gravity[0], self._gravity[1], self._gravity[2])
        p.setAdditionalSearchPath(pybullet_data.getDataPath())  #  PyBullet_data package (see doc)       
        self._ID = []
        self._ID.append(p.loadURDF("plane.urdf"))        
        
    def step(self):
        p.stepSimulation()
        t.sleep(self._timedelta)
        
    def load_aggregates(self, aggregate_positions, aggregate_urdf):        
        for pos in aggregate_positions:                          
            self._ID.append(p.loadURDF(aggregate_urdf, basePosition=pos))            
            
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