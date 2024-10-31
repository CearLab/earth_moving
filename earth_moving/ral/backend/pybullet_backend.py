import pybullet as p
from earth_moving.ral.backend.base_backend import BaseBackend, BaseSensorBackend

class PybulletBackend(BaseBackend):
    
    def __init__(self, timedelta) -> None:
        super().__init__()
        self._timedelta

    def initiate_rgb_sensor(self) -> BaseSensorBackend: # TODO: unlike ROS, this needs to happen for all sensors that we want at the beginning of the run
        class PybulletSensorBackend(BaseSensorBackend):
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
                return img
        sensor_backend = PybulletSensorBackend()
        return sensor_backend