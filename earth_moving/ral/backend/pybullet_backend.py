import pybullet as p
from sensor.sensor_backend import BaseSensorBackend
from sensor.sensor_rgb import SensorRGBBackend
from backend.base_backend import BaseBackend

class PybulletBackend(BaseBackend):
    
    def __init__(self, timedelta) -> None:
        super().__init__()
        self._timedelta = timedelta               

    # def initiate_rgb_sensor(self) -> BaseSensorBackend: # TODO: unlike ROS, this needs to happen for all sensors that we want at the beginning of the run
        
    class PybulletSensorRGBBackend(SensorRGBBackend):
        def __init__(self,**kwargs) -> None:
            super().__init__(kwargs)
            self._img = None
        
        def get_data(self):
            self._img = p.getCameraImage(self._imgW,self._imgH)
            
            
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
                
        # sensor_backend = PybulletSensorRGBBackend()
        # return sensor_backend