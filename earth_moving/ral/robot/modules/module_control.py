import numpy as np

class Controller:
    def __init__(self, **kwargs) -> None:
        
        controller = kwargs.get('controller')
        self._name = controller.get('name')
        
        if self._name == 'PID':
            self._Kp = controller.get('Kp')
            self._Ki = controller.get('Ki')
            self._Kd = controller.get('Kd')            
            self._max = controller.get('max')
            self._integral_reset_interval = controller.get('integral_reset_interval')
            self._accuracy = controller.get('accuracy')
            self._dimension = controller.get('dimension')
            
            self._status = 'IDLE'
            self._error =           np.zeros(self._dimension)
            self._integral =        np.zeros(self._dimension)
            self._derivative =      np.zeros(self._dimension)
            self._previous_error =  np.zeros(self._dimension)
            self._output =          np.zeros(self._dimension)
            self._dt = None
            
            self._error_store = []
            self._integral_store = []
            self._derivative_store = []
            self._output_store = []
            
    def compute_error(self, setpoint: float, current_value: float) -> None:                        
        self._previous_error = self._error
        self._error = setpoint - current_value
        self._error_store.append(self._error)
            
    def compute_integral(self) -> None:
        self._integral += self._error * self._dt
        self._integral_store.append(self._integral)
        
    def compute_derivative(self) -> None:
        self._derivative = (self._error - self._previous_error) / self._dt
        self._derivative_store.append(self._derivative)
        
    def reset_integral(self) -> None:
        if isinstance(self._integral_reset_interval, int):
            if len(self._integral_store) % self._integral_reset_interval == 0:
                self._integral = 0.0
        
    def compute_output(self) -> None:
        control = self._Kp * self._error + self._Ki * self._integral + self._Kd * self._derivative
        self._output = np.clip(control, -self._max, self._max)
        self._output_store.append(self._output)
        
    def PID_controller(self, setpoint: float, current_value: float) -> None:
        # convert values type
        setpoint = np.array(setpoint)
        current_value = np.array(current_value)
        
        # control
        self.compute_error(setpoint, current_value)
        self.compute_integral()
        self.compute_derivative()
        self.reset_integral()
        self.compute_output()
        
    def step(self, setpoint: float, current_value: float) -> None:
        if self._status == 'FINISHED':
            self._status = 'IDLE'
            return
        
        if self._name == 'PID':
            self.PID_controller(setpoint, current_value)
        
        # check accuracy and update status
        if all(abs(self._error) <= self._accuracy):
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
        u = np.array([self._force])
        
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
        
        
    
        