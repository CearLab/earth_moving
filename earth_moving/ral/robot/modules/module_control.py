import numpy as np
import earth_moving.ral.algorithms.module_misc as module_misc

class Controller:
    def __init__(self, **kwargs) -> None:
        
        controller = kwargs.get('controller')
        self._name = controller.get('name')
        
        self._max = controller.get('max')            
        self._accuracy = controller.get('accuracy')
        self._dimension = controller.get('dimension') 
        self._visits_reset = controller.get('visits_reset') 
        
        if self._name == 'P':            
            self._Kp = controller.get('Kp')                        
                                    
        if self._name == 'P_repulsive':
            self._Kp = controller.get('Kp')                        
            self._map_repulsive = controller.get('map_repulsive')
            self._neighbourhood_size = controller.get('neighbourhood_size')            
            self._beta_repulsive = controller.get('beta_repulsive')
            self._alpha_memory = controller.get('alpha_memory')
            self._vegetation_barrier = controller.get('vegetation_barrier')
            self._river_barrier = controller.get('river_barrier')       
            
            # attributes
            self._variance = 0
            self._previous_control = np.zeros((1, self._dimension))                              
                        
        self._status = 'IDLE'        
        self._dt = None
        self._current_value = None
        self._setpoint = None
        self._error = None
        self._output = None
                
            
    def compute_error(self, setpoint: float, current_value: float) -> None:                
        
        # error
        if self._name == 'P':
            error = setpoint - current_value
        elif self._name == 'P_repulsive':
            error = []
            for pos in current_value:
                error.append(setpoint - pos)
        else:
            raise NotImplementedError()
        
        # store info
        self._current_value = current_value
        self._setpoint = setpoint
        self._error = error
        
        return error
        
    def compute_output(self, error, neighbourhood = None) -> None:                
        
        # compute the control signal
        if self._name == 'P' or np.linalg.norm(error) < 2e0:   
            control = self._Kp * error
            
        elif self._name == 'P_repulsive':            
                        
            # Compute the repulsive potential based on the neighborhood            
            neighbourhood_pos = neighbourhood[0]
            neighbourhood_values = neighbourhood[1]
            
            # normalize values
            min_value = min(neighbourhood_values)
            max_value = max(neighbourhood_values)
            neighbourhood_values = neighbourhood_values / max_value
            
            # error at the current position
            error_center = self._setpoint - self._current_value[-1]
            err_max = max(np.linalg.norm(np.asarray(error), axis=1))
            
            # init
            value_list = []
            
            # balance between attractive and repulsive potential
            beta = self._beta_repulsive
            
            # far, repulsive potential
            if np.linalg.norm(error_center) > 2e0:
                for pos, val, err in zip(neighbourhood_pos, neighbourhood_values, error):                                
                    if not(pos[0] == self._current_value[-1][0] and pos[1] == self._current_value[-1][1]):                                                   
                        control_attractive = self._Kp * np.linalg.norm(err) / err_max
                        value = (1 - beta) * control_attractive + beta * val
                        value_list.append([pos[0], pos[1], value])
                
                # Find the position corresponding to the minimum value in the third column
                value_array = np.array(value_list)
                min_index = np.argmin(value_array[:, 2])
                control_x=  value_array[min_index, 0] - self._current_value[-1][0]
                control_y = value_array[min_index, 1] - self._current_value[-1][1]                
            # near, attractive potential
            else:                
                control_x = self._Kp * error_center[0]
                control_y = self._Kp * error_center[1]
                    
            # Set control
            control = np.array([control_x, control_y])       
            
            # Define range of alpha (e.g., between 0.2 and 0.9)
            alpha_min = self._alpha_memory[0]
            alpha_max = self._alpha_memory[1]
            
            # variance of the neighbourhood                        
            variance_max = max(np.var(neighbourhood_values), self._variance) + 1e-4
            self._variance = np.var(neighbourhood_values) + 1e-4
            normalized_variance = self._variance / variance_max
            # Inverse relationship: higher variance → lower alpha
            alpha = alpha_max - (alpha_max - alpha_min) * normalized_variance
            # Apply the alpha value to the control signal
            control = alpha * control + (1 - alpha) * self._previous_control
                                    
        else:
            raise NotImplementedError()
            
        _output = np.clip(control, -self._max, self._max)
        _output = _output.reshape((1, self._dimension))
        return _output
        
    def P_controller(self, setpoint: float, current_value: float) -> None:
        # convert values type
        setpoint = np.array(setpoint)
        current_value = np.array(current_value)
        
        # control
        error = self.compute_error(setpoint, current_value)
        output = self.compute_output(error)
        
        return output
    
    def P_repulsive_controller(self, setpoint: float, current_value: float, _neighbourhood_values: list) -> None:
        # convert values type
        setpoint = np.array(setpoint)
        current_value = np.array(current_value)
        pos_values = np.array(_neighbourhood_values[0])                      
        
        # control
        error = self.compute_error(setpoint, pos_values)
        output = self.compute_output(error, _neighbourhood_values)
        
        return output
        
    def step(self, setpoint: float, current_value: float, _neighbourhood = None) -> None:
        if self._status == 'FINISHED':
            self._status = 'IDLE'
            return
        
        if self._name == 'P':
            self._output = self.P_controller(setpoint, current_value)
            error_check = np.linalg.norm(self._error)
        elif self._name == 'P_repulsive':
            self._output = self.P_repulsive_controller(setpoint, current_value, _neighbourhood)
            error_check = np.linalg.norm(self._error[-1])
        
        # check accuracy and update status
        if error_check <= self._accuracy:
            self._status = 'FINISHED'         
        else:
            self._status = 'ACTIVE'
            
class Dynamics:
    def __init__(self, initial_state, **kwargs) -> None:
        
        dynamics = kwargs.get('dynamics')
        self._name = dynamics.get('name')
        
        if self._name == 'integrator':
            self._mass = dynamics.get('mass')
            self._friction = dynamics.get('friction')
            self._status = 'IDLE'
            
        self._dt = None
        self._state = initial_state
                
        self._force = []
        self._output = []        
            
    def SS_dynamics(self) -> None:        
        # closed form discrete dynamics
        A = np.array([[1, self._dt], [0, 1 - self._friction/self._mass]])
        B = np.array([[0], [self._dt/self._mass]])
        C = np.array([[1, 0]])
        D = np.array([[0]])
        
        # control input
        u = np.array(self._force)
        
        # trick to stop the state - hybrid system
        for i, control in enumerate(u[0]):
            if control == 0:
                self._state[1][i] = 0
        
        # state update
        self._state = np.dot(A, self._state) + np.dot(B, u)
        self._output = np.dot(C, self._state) + np.dot(D, u)
        
    def step(self, input) -> None:
        if self._name == 'integrator':
            self._force = input
            self.SS_dynamics()
        else:
            raise NotImplementedError()
        
        
    
        