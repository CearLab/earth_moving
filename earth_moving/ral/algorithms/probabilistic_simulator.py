import numpy as np
from scipy.spatial.transform import Rotation as R
from abc import ABC, abstractmethod
from scipy.interpolate import interp1d, interp2d

class ProbabilisticSimulator(ABC):
    
    def __init__(self, **kwargs) -> None:
        pass
        
    def define_shovel(self, **kwargs) -> None:        
        self._shovel_name = kwargs.get('name')
        self._shovel_width = kwargs.get('width')
        self._shovel_height = kwargs.get('height')
        
    def define_trajectory(self, **kwargs) -> None:        
        self._trajectory_length = kwargs.get('trajectory_length')
        self._trajectory_curvature = kwargs.get('trajectory_curvature')
        self._trajectory_points = kwargs.get('trajectory_points')
        self._start_pos = kwargs.get('start_pos')
        self._start_orientation = R.from_euler('xyz', kwargs.get('start_orientation'), degrees=False).as_quat()        
        
    def generate_trajectory(self):
        trajectory_pos = []        
        trajectory_orientation = []
        radius = self._trajectory_curvature
        initial_orientation = R.from_quat(self._start_orientation).as_euler('xyz', degrees=False)
        orientation = R.from_euler('xyz', [0, 0, initial_orientation[2]], degrees=False).as_quat()        
        
        if radius == 0:
            # Straight line
            for i in range(self._trajectory_points):
                x = self._start_pos[0] + i * (self._trajectory_length / self._trajectory_points)
                y = self._start_pos[1] 
                z = self._start_pos[2]
                trajectory_pos.append([x, y, z])    
                trajectory_orientation.append(orientation)
        else:
            # Curved path
            total_angle = self._trajectory_length / radius
            angle_step =  total_angle / self._trajectory_points
            for i in range(self._trajectory_points):
                angle = i * angle_step
                x = self._start_pos[0] + radius * np.sin(angle)
                y = self._start_pos[1] + radius * (np.cos(angle) - 1)
                z = self._start_pos[2]
                trajectory_pos.append([x, y, z]) 
                orientation = R.from_euler('xyz', [0, 0, initial_orientation[2] - angle], degrees=False).as_quat()
                trajectory_orientation.append(orientation)
                
        # Interpolate positions
        trajectory_pos = np.asarray(trajectory_pos).reshape(-1, 3)        
        self._trajectory_interp_func_x = interp1d(np.arange(trajectory_pos.shape[0]), 
                                                          trajectory_pos[:,1], 
                                                          kind='linear',                                                           
                                                          fill_value="extrapolate")
        
        self._trajectory_interp_func_y = interp1d(np.arange(trajectory_pos.shape[0]), 
                                                          trajectory_pos[:,2], 
                                                          kind='linear',                                                           
                                                          fill_value="extrapolate")

        # Interpolate orientations
        trajectory_orientation = np.array(trajectory_orientation)
        self._trajectory_orientation_interp_func = interp1d(np.arange(trajectory_orientation.shape[0]), 
                                                            trajectory_orientation[:,2], 
                                                            kind='linear',                                                            
                                                            fill_value="extrapolate")
        
        return trajectory_pos, trajectory_orientation
    
    def define_shovel_area(self, position, orientation):
                
        corners_sequence = []         
        for i in range(len(position)):
            pos = position[0]
            corners = [
                        [pos[0], pos[1] - self._shovel_width * 0.5, pos[2]],
                        [pos[0], pos[1] + self._shovel_width * 0.5, pos[2]],
                        [pos[0] + self._shovel_height * 0.5, pos[1] + self._shovel_width * 0.5, pos[2]],
                        [pos[0] + self._shovel_height * 0.5, pos[1] - self._shovel_width * 0.5, pos[2]]
                    ]
            rotation_matrix = R.from_quat(orientation[i]).as_matrix()            
            rotated_corners = [np.dot(rotation_matrix, corner).tolist() for corner in corners]
            rotated_corners = (np.array(rotated_corners) + np.array(position[i])).tolist()            
            corners_sequence += rotated_corners
        return corners_sequence
    
    def define_prediction_area(self):
        
        # init        
        corners_L_pos_left = np.zeros([self._trajectory_points, 3])
        corners_L_pos_right = np.zeros([self._trajectory_points, 3])
        corners_L_pos_left_straight = np.zeros([3])
        corners_L_pos_right_straight = np.zeros([3])
        corners_C_pos_left = np.zeros([self._trajectory_points, 3])
        corners_C_pos_right = np.zeros([self._trajectory_points, 3])        
        corners_C_pos_left_straight = np.zeros([3])
        corners_C_pos_right_straight = np.zeros([3])
        corners_R_pos_left = np.zeros([self._trajectory_points, 3])
        corners_R_pos_right = np.zeros([self._trajectory_points, 3])
        corners_R_pos_left_straight = np.zeros([3])
        corners_R_pos_right_straight = np.zeros([3])
        corners_T_pos_left = np.zeros([self._trajectory_points, 3])
        corners_T_pos_right = np.zeros([self._trajectory_points, 3])
        corners_T_pos_left_straight = np.zeros([3])
        corners_T_pos_right_straight = np.zeros([3])
        
        # central area       
        # Find the position relative to a trajectory length of _shovel_width
        steps = np.linspace(0.0, self._shovel_width , self._trajectory_points)        
        for i in range(len(steps)):            
            index = steps[i] / self._trajectory_length * (self._trajectory_points - 1)
                        
            corners_C_pos_left_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_C_pos_left_straight[1] = self._trajectory_interp_func_y(index) + self._shovel_width * 0.5
            corners_C_pos_left_straight[2] = 0.0
            
            corners_C_pos_right_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_C_pos_right_straight[1] = self._trajectory_interp_func_y(index) - self._shovel_width * 0.5
            corners_C_pos_right_straight[2] = 0.0
            
            yaw = self._trajectory_orientation_interp_func(index)
            angle = np.array([0, 0, yaw])
            corners_C_rotation = R.from_euler('xyz',angle, degrees=False).as_matrix()
            corners_C_pos_left[i][:]  = np.dot(corners_C_rotation, corners_C_pos_left_straight)
            corners_C_pos_right[i][:] = np.dot(corners_C_rotation, corners_C_pos_right_straight)            
        
        # stack as lists
        corners_C_pos_left = [[corners_C_pos_left[i][0], corners_C_pos_left[i][1], corners_C_pos_left[i][2]] for i in range(self._trajectory_points)]
        corners_C_pos_right = [[corners_C_pos_right[i][0], corners_C_pos_right[i][1], corners_C_pos_right[i][2]] for i in range(self._trajectory_points)]        
        corners_C_pos_right = corners_C_pos_right[::-1]
        corners_C = corners_C_pos_left + corners_C_pos_right
                
        # left area
        # Find the position relative to a trajectory length of _shovel_width
        steps = np.linspace(0.0, self._shovel_width , self._trajectory_points)
        for i in range(len(steps)):            
            index = steps[i] / self._trajectory_length * (self._trajectory_points - 1)
                        
            corners_L_pos_left_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_L_pos_left_straight[1] = self._trajectory_interp_func_y(index) + self._shovel_width * 0.5
            corners_L_pos_left_straight[2] = 0.0
            
            corners_L_pos_right_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_L_pos_right_straight[1] = self._trajectory_interp_func_y(index) + self._shovel_width * 1.5
            corners_L_pos_right_straight[2] = 0.0
            
            yaw = self._trajectory_orientation_interp_func(index)
            angle = np.array([0, 0, yaw])
            corners_L_rotation = R.from_euler('xyz',angle, degrees=False).as_matrix()
            corners_L_pos_left[i][:]  = np.dot(corners_L_rotation, corners_L_pos_left_straight)
            corners_L_pos_right[i][:] = np.dot(corners_L_rotation, corners_L_pos_right_straight)
            
        # stack as lists
        corners_L_pos_left = [[corners_L_pos_left[i][0], corners_L_pos_left[i][1], corners_L_pos_left[i][2]] for i in range(self._trajectory_points)]
        corners_L_pos_right = [[corners_L_pos_right[i][0], corners_L_pos_right[i][1], corners_L_pos_right[i][2]] for i in range(self._trajectory_points)]
        corners_L_pos_right = corners_L_pos_right[::-1]
        corners_L = corners_L_pos_left + corners_L_pos_right
        
        # right area
        # Find the position relative to a trajectory length of _shovel_width
        steps = np.linspace(0.0, self._shovel_width , self._trajectory_points)
        for i in range(len(steps)):            
            index = steps[i] / self._trajectory_length * (self._trajectory_points - 1)
                        
            corners_R_pos_left_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_R_pos_left_straight[1] = self._trajectory_interp_func_y(index) - self._shovel_width * 1.5
            corners_R_pos_left_straight[2] = 0.0
            
            corners_R_pos_right_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_R_pos_right_straight[1] = self._trajectory_interp_func_y(index) - self._shovel_width * 0.5
            corners_R_pos_right_straight[2] = 0.0
            
            yaw = self._trajectory_orientation_interp_func(index)
            angle = np.array([0, 0, yaw])
            corners_R_rotation = R.from_euler('xyz',angle, degrees=False).as_matrix()
            corners_R_pos_left[i][:]  = np.dot(corners_R_rotation, corners_R_pos_left_straight)
            corners_R_pos_right[i][:] = np.dot(corners_R_rotation, corners_R_pos_right_straight)
            
        # stack as lists
        corners_R_pos_left = [[corners_R_pos_left[i][0], corners_R_pos_left[i][1], corners_R_pos_left[i][2]] for i in range(self._trajectory_points)]
        corners_R_pos_right = [[corners_R_pos_right[i][0], corners_R_pos_right[i][1], corners_R_pos_right[i][2]] for i in range(self._trajectory_points)]
        corners_R_pos_right = corners_R_pos_right[::-1]
        corners_R = corners_R_pos_left + corners_R_pos_right
        
        # top area
        # Find the position relative to a trajectory length of _shovel_width
        steps = np.linspace(self._shovel_width, self._shovel_width * 3 , self._trajectory_points)
        print('steps', steps)
        for i in range(len(steps)):            
            index = steps[i] / self._trajectory_length * (self._trajectory_points - 1)
                        
            corners_T_pos_left_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_T_pos_left_straight[1] = self._trajectory_interp_func_y(index) + self._shovel_width * 1.5
            corners_T_pos_left_straight[2] = 0.0
            
            corners_T_pos_right_straight[0] = self._trajectory_interp_func_x(index) + steps[i]
            corners_T_pos_right_straight[1] = self._trajectory_interp_func_y(index) - self._shovel_width * 1.5
            corners_T_pos_right_straight[2] = 0.0
            
            yaw = self._trajectory_orientation_interp_func(index)
            angle = np.array([0, 0, yaw])
            corners_T_rotation = R.from_euler('xyz',angle, degrees=False).as_matrix()
            corners_T_pos_left[i][:]  = np.dot(corners_T_rotation, corners_T_pos_left_straight)
            corners_T_pos_right[i][:] = np.dot(corners_T_rotation, corners_T_pos_right_straight)
            
        # stack as lists
        corners_T_pos_left = [[corners_T_pos_left[i][0], corners_T_pos_left[i][1], corners_T_pos_left[i][2]] for i in range(self._trajectory_points)]
        corners_T_pos_right = [[corners_T_pos_right[i][0], corners_T_pos_right[i][1], corners_T_pos_right[i][2]] for i in range(self._trajectory_points)]
        corners_T_pos_right = corners_T_pos_right[::-1]
        corners_T = corners_T_pos_left + corners_T_pos_right
        
        
        # stack areas vertically
        corners = [corners_L, corners_C, corners_R, corners_T]
        return corners
    
    # define simulator ptobabilities
    def define_probabilities(self) -> None:
    
        # consider Klen      = 1 (see paper)
        # consider traj      = [tight_left, left, straight, right, tight_right] (see paper)
        # related curvatures
        x = np.array([-0.5, -0.3, 10, 0.3, 0.5])        
        
        # data from paper
        self.P_left_region =    np.array([0.02,  0.03,   0.02,   0.09,   0.18])
        self.P_center_region =  np.array([0.0,   0.0,    0.0,    0.0,    0.00])
        self.P_right_region =   np.array([0.18,  0.17,   0.06,   0.11,   0.03])
        self.P_top_region =     np.array([0.80,  0.80,   0.92,   0.80,   0.79])
        
        # Interpolation        
        self.interp_left_region = interp1d(x, self.P_left_region, kind='linear', fill_value="extrapolate")
        self.interp_center_region = interp1d(x, self.P_center_region, kind='linear', fill_value="extrapolate")
        self.interp_right_region = interp1d(x, self.P_right_region, kind='linear', fill_value="extrapolate")
        self.interp_top_region = interp1d(x, self.P_top_region, kind='linear', fill_value="extrapolate")
        
    # compute probabilities
    def compute_probabilities(self):
        
        # compute probabilities
        probabilities = []        
        x = self._trajectory_curvature
        probabilities = np.array((self.interp_left_region(x), self.interp_center_region(x), self.interp_right_region(x), self.interp_top_region(x)))
        
        return probabilities
    