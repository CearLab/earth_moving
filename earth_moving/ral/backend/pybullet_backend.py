import pybullet as p
import pybullet_data
import time as t
import os
import matplotlib.pyplot as plt
import numpy as np

from ral.backend.base_backend import BaseBackend
from ral.sensor.sensor_backend import BaseSensorBackend
from ral.robot.robot_backend import BaseRobotBackend
from ral.algorithms.probabilistic_simulator import ProbabilisticSimulator


class PybulletBackend(BaseBackend):
    
    def __init__(self, **kwargs) -> None:
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._update_period = simulation.get('update_period')
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
        
        self._ID_aggregates = []
        self._current_time = 0
        
    def step(self):
        p.stepSimulation()        
        self._current_time += self._timedelta
        t.sleep(self._timedelta)
        
    def load_aggregates(self, aggregate_positions, aggregate_urdf):        
        for pos in aggregate_positions:                          
            self._ID.append(p.loadURDF(aggregate_urdf, basePosition=pos)) 
            self._ID_aggregates.append(self._ID[-1])       
            
    def get_aggregates_positions(self):
        positions = []
        for ID in self._ID_aggregates:
            positions.append(p.getBasePositionAndOrientation(ID)[0])
        return positions
    
    def color_aggregates_in_clusters(self, labels):
        
        n_clusters = len(np.unique(labels))
        colors = plt.cm.get_cmap('tab10', n_clusters)  # Generate a colormap with `n_clusters` distinct colors
        
        for i, aggregate_id in enumerate(self._ID_aggregates):
            color = colors(labels[i])[:3]  # Get the RGB values for the label
            p.changeVisualShape(aggregate_id, -1, rgbaColor=list(color) + [1])
                
    def draw_gaussians(self, points_list):

        n_clusters = len(points_list)
        colors = plt.cm.get_cmap('tab10', n_clusters)  # Generate a colormap with `n_clusters` distinct colors       
                    
        for  i in range(n_clusters):
            color = [colors(i)[:3]] * len(points_list[0])
            points = points_list[i]
            points = np.asarray(points)
            points[:, 2] /= 1*np.max(points[:, 2])
            points = points.tolist()
            p.addUserDebugPoints(points, list(color), 2)
            
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
    
    def initiate_robot(self,**kwargs):
        
        class PybulletShovelBackend(BaseRobotBackend,ProbabilisticSimulator):
            
            def __init__(self,**kwargs) -> None:
                super().__init__(**kwargs)
                super().initiate_robot(**kwargs)
                super().define_shovel(**self._robot)
                super().define_trajectory(**self._robot) 
                super().define_clusters(**self._robot)
                super().define_probabilities()
                self._probabilities = self.compute_probabilities()                           
            
            def draw_shovel(self, corners) -> None:  
                                                               
                for i in range(len(corners)):
                    next_i = (i + 1) % len(corners)                    
                    p.addUserDebugLine(corners[i], corners[next_i], [0, 0, 0], 4)
            
            def draw_trajectory(self, corners) -> None:
                
                grouped_corners = [corners[i:i + 4] for i in range(0, len(corners), 4)] # TODO: this is not general. If the shovel has 5 corners it changes
                for group in grouped_corners:                    
                    self.draw_shovel(group)                
                    
            def draw_prediction_area(self, corners) -> None:                                                
                                
                # sequence of colors every 4 corners = [red, green, blue, yellow]                
                for i in range(len(corners)):                    
                    color = [1, 0, 0] if i % 4 == 0 else [0, 1, 0] if i % 4 == 1 else [0, 0, 1] if i % 4 == 2 else [1, 1, 0]                
                    for j in range(len(corners[i])):
                        next_j = (j + 1) % len(corners[i])
                        p.addUserDebugLine(corners[i][j], corners[i][next_j], color, 8)                        
                
            def draw_probabilities(self,corners) -> None:
                
                for i in range(len(self._probabilities)):
                    color = [1, 0, 0] if i % 4 == 0 else [0, 1, 0] if i % 4 == 1 else [0, 0, 1] if i % 4 == 2 else [1, 1, 0]
                    base_corners = corners[i]                                                                                                            
                    barycenter = np.mean(base_corners, axis=0)
                    
                    x_points = np.linspace(min(base_corners)[0], max(base_corners)[0], num=20)
                    y_points = np.linspace(min(base_corners)[1], max(base_corners)[1], num=20)
                    grid_points = np.array(np.meshgrid(x_points, y_points)).T.reshape(-1, 2)
                    
                    points_cluster = []
                    for point in grid_points:
                        points_cluster.append([point[0], point[1], self._probabilities[i]])
                    
                    p.addUserDebugText(f'{self._probabilities[i]:.2f}', barycenter, color, textSize=2)
                    color_list = [color] * len(points_cluster)
                    p.addUserDebugPoints(points_cluster, color_list, 2)
                    
                    
                                                                     
                    
                
        shovel_backend = PybulletShovelBackend(**kwargs)
        return shovel_backend        