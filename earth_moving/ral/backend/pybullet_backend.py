import pybullet as p
import pybullet_data
import time as t
import os
import matplotlib.pyplot as plt
import numpy as np
from ral.sensor.sensor_backend import BaseSensorBackend
from ral.backend.base_backend import BaseBackend

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
        self._ID.append(p.loadURDF('r2d2.urdf', basePosition=[0.0, 0.0, 1]))        
        
    def step(self):
        p.stepSimulation()
        t.sleep(self._timedelta)
            

    def initiate_rgb_sensor(self,**kwargs) -> BaseSensorBackend: # TODO: unlike ROS, this needs to happen for all sensors that we want at the beginning of the run
        
        class PybulletSensorRGBBackend(BaseSensorBackend):
            def __init__(self,**kwargs) -> None:
                super().__init__(**kwargs)
                self._imgW = self._sensor.get('imgW')
                self._imgH = self._sensor.get('imgH')
                
                self._camera_pose = self._sensor.get('camera_pose')
                self._target_pose = self._sensor.get('target_pose')
                self._camera_up_vector = self._sensor.get('camera_up_vector')
                self._viewMatrix = p.computeViewMatrix(self._camera_pose, self._target_pose, self._camera_up_vector)
                
                self._fov = self._sensor.get('fov')
                self._aspect = self._imgW / self._imgH
                self._near = self._sensor.get('near')
                self._far = self._sensor.get('far')
                self._projectionMatrix = p.computeProjectionMatrixFOV(self._fov, self._aspect, self._near, self._far)
            
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
    
    def initiate_imu_sensor(self) -> BaseSensorBackend:   
        raise NotImplementedError() 
    
    def initiate_gyro_sensor(self) -> BaseSensorBackend:   
        raise NotImplementedError()  
            
        '''class PybulletSensorRGBBackend(SensorRGBBackend):
            def __init__(self, **kwargs) -> None:
                super().__init__(kwargs)
                self._imgW = self._kwargs.get('imgW')
                self._imgH = self._kwargs.get('imgH')                
                self._camera_pose_method = self._kwargs.get('camera_pose_method')
                if type(self._camera_pose_method) is callable:
                    self.timing_law = ...
                else:
                    self._robotBodyUniqueId = ... # find the robot unique ID
                    self._EELinkIndex = ... # find the EE link index
            def get(self):
                if type(self._camera_pose_method) is callable:
                    viewMatrix, projectionMatrix = ... # infer from timing law
                else:                
                    linkWorldPosition, linkWorldOrientation = p.getLinkState(self._bodyUniqueId, self._linkIndex)
                    viewMatrix, projectionMatrix = f(linkWorldPosition, linkWorldOrientation)
                img = p.getCameraImage(self._imgW, self._imgH, 
                                       viewMatrix, projectionMatrix, renderer=p.ER_BULLET_HARDWARE_OPENGL) # TODO: what if camera is mounted on the robot?
                return img'''
                
        