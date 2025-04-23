import numpy as np
import earth_moving.ral.algorithms.module_misc as module_misc

class Controller:
    def __init__(self, **kwargs) -> None:
        
        controller = kwargs.get('controller')
        self._name = controller.get('name')
        
        if self._name == 'P':
            
            self._Kp = controller.get('Kp')                        
                                    
        if self._name == 'P_repulsive':
            self._Kp = controller.get('Kp')                        
            self._Kp_repulsive = controller.get('Kp_repulsive')
                                    
        self._max = controller.get('max')            
        self._accuracy = controller.get('accuracy')
        self._dimension = controller.get('dimension')                        
                        
        self._status = 'IDLE'        
        self._dt = None
        self._current_value = None
        self._setpoint = None
        self._error = None
        self._output = None
                
            
    def compute_error(self, setpoint: float, current_value: float) -> None:                
        
        # error
        error = setpoint - current_value
        
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
            control_attractive = self._Kp * np.linalg.norm(self._error)
                        
            # Compute the repulsive potential based on the neighborhood
            control_repulsive_x = 0
            control_repulsive_y = 0
            neighbourhood_pos = neighbourhood[0]
            neighbourhood_values = neighbourhood[1]
            val_min = np.nanmin(neighbourhood_values)
            for val, pos in zip(neighbourhood_values, neighbourhood_pos):
                if not np.isnan(val):
                    distance = np.linalg.norm(self._current_value - pos)
                    if distance > 1e-4:  # Avoid division by zero
                        attraction_factor = 1 / (1 + val - val_min)
                        control_repulsive_x += attraction_factor * (pos[0] - self._current_value[0]) / (distance**3)
                        control_repulsive_y += attraction_factor * (pos[1] - self._current_value[1]) / (distance**3)

            # Combine attractive and repulsive components
            control_x = control_attractive * (self._setpoint[0] - self._current_value[0]) / (np.linalg.norm(self._error) + 1e-8) + \
                self._Kp_repulsive * control_repulsive_x
            control_y = control_attractive * (self._setpoint[1] - self._current_value[1]) / (np.linalg.norm(self._error) + 1e-8) + \
                self._Kp_repulsive * control_repulsive_y
                    
            # Set control
            control = np.array([control_x, control_y])                            
                                    
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
        
        # control
        error = self.compute_error(setpoint, current_value)
        output = self.compute_output(error, _neighbourhood_values)
        
        return output
        
    def step(self, setpoint: float, current_value: float, _neighbourhood = None) -> None:
        if self._status == 'FINISHED':
            self._status = 'IDLE'
            return
        
        if self._name == 'P':
            self._output = self.P_controller(setpoint, current_value)
        elif self._name == 'P_repulsive':
            self._output = self.P_repulsive_controller(setpoint, current_value, _neighbourhood)
        
        # check accuracy and update status
        if (abs(np.asarray(self._error)) <= self._accuracy).all():
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
        
        
    
        