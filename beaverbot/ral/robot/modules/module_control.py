import numpy as np

class Controller:
    """Multi-purpose controller: P (proportional) and P_repulsive (with potential field obstacle avoidance)."""
    
    def __init__(self, **kwargs) -> None:
        controller = kwargs.get('controller')
        self._name = controller.get('name')
        self._max = controller.get('max')
        self._accuracy = controller.get('accuracy')
        self._dimension = controller.get('dimension')
        
        self._Kp = controller.get('Kp')
        self._Kd = controller.get('Kd')
        self._Ki = controller.get('Ki')
        
        if self._name == 'P_repulsive':
            self._map_repulsive = controller.get('map_repulsive')
            self._neighbourhood_size = controller.get('neighbourhood_size')
            self._beta_repulsive = controller.get('beta_repulsive')
            self._alpha_memory = controller.get('alpha_memory')
            self._vegetation_barrier = controller.get('vegetation_barrier')
            self._river_barrier = controller.get('river_barrier')
            self._variance = 0
            self._previous_control = np.zeros((1, self._dimension))
        
        self._Kp_init, self._Kd_init, self._Ki_init = self._Kp, self._Kd, self._Ki
        self._status = 'IDLE'
        self._dt = None
        self._max_scaled = None
        self._current_value = None
        self._setpoint = None
        self._error = 0.0
        self._error_old = 0.0
        self._error_integral = 0.0
        self._output = None
    
    def compute_error(self, setpoint: float, current_value: float) -> None:
        """Compute control error (setpoint - current_value) and update internal state."""
        error = setpoint - (current_value[-1] if self._name == 'P_repulsive' else current_value)
        
        self._current_value = current_value
        self._setpoint = setpoint
        self._error_old = self._error
        self._error = error
        self._error_integral += self._error
        self._max_scaled = self._max
        return error
    
    def compute_output(self, error, neighbourhood=None) -> None:
        """Compute control output: P uses Kp*error, P_repulsive combines attractive/repulsive forces from neighbourhood."""
        if self._name == 'P':
            error_d = error - self._error_old
            control = self._Kp * error + self._Kd * error_d + self._Ki * self._error_integral
        
        elif self._name == 'P_repulsive':
            # Normalize neighbourhood values
            neighbourhood_pos, neighbourhood_values = neighbourhood[0], neighbourhood[1]
            min_val, max_val = np.min(neighbourhood_values), np.max(neighbourhood_values)
            if max_val > min_val:
                neighbourhood_values = (neighbourhood_values - min_val) / (max_val - min_val) * np.linalg.norm(error)
            else:
                neighbourhood_values = np.zeros_like(neighbourhood_values)
            
            # Compute repulsive potential when far from setpoint
            potential_array = []
            if np.linalg.norm(error) > 4.0:
                value_list = []
                for pos, val in zip(neighbourhood_pos, neighbourhood_values):
                    is_current = (pos[0] == self._current_value[-1][0] and pos[1] == self._current_value[-1][1])
                    value_list.append([pos[0], pos[1], np.inf if is_current else val])
                
                value_array = np.array(value_list)
                finite_mask = np.isfinite(value_array[:, 2])
                finite_array = value_array[finite_mask]
                value_pos = value_array[finite_mask][0, 2] if (finite_mask).any() else 0
                
                for i in range(finite_array.shape[0]):
                    potential = (value_pos - finite_array[i, 2]) * (self._current_value[-1] - finite_array[i, :2])
                    potential_array.append(potential)
            else:
                potential_array = [np.array((0.0, 0.0))]
            
            # PID control
            error_d = error - self._error_old
            control_attractive = self._Kp * error + self._Kd * error_d + self._Ki * self._error_integral
            control_repulsive = self._Kp * np.sum(potential_array, axis=0)
            control = self._beta_repulsive * control_repulsive + (1 - self._beta_repulsive) * control_attractive
        
        else:
            raise NotImplementedError()
        
        return np.clip(control, -self._max_scaled, self._max_scaled).reshape((1, self._dimension))
    
    def P_controller(self, setpoint: float, current_value: float) -> None:
        """Simple P controller: output = Kp * (setpoint - current_value)."""
        return self.compute_output(self.compute_error(np.array(setpoint), np.array(current_value)))
    
    def P_repulsive_controller(self, setpoint: float, current_value: float, _neighbourhood_values: list) -> None:
        """P controller with repulsive potential field for obstacle avoidance using neighbourhood values."""
        setpoint, current_value = np.array(setpoint), np.array(current_value)
        pos_values = np.array(_neighbourhood_values[0])
        return self.compute_output(self.compute_error(setpoint, pos_values), _neighbourhood_values)
    
    def step(self, setpoint: float, current_value: float, _neighbourhood=None) -> None:
        """Execute one control step, update status based on error convergence (IDLE/ACTIVE/FINISHED)."""
        if self._status == 'FINISHED':
            self._status = 'IDLE'
            return
        
        if self._name == 'P':
            self._output = self.P_controller(setpoint, current_value)
        elif self._name == 'P_repulsive':
            self._output = self.P_repulsive_controller(setpoint, current_value, _neighbourhood)
        
        self._status = 'FINISHED' if np.linalg.norm(self._error) <= self._accuracy else 'ACTIVE'


class Dynamics:
    """System dynamics simulator for robotic motion (integrator: double integrator with friction)."""
    
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
        """State-space dynamics: x[k+1] = A*x[k] + B*u[k], y[k] = C*x[k]. Hybrid system stops velocity on zero input."""
        dt = self._dt
        A = np.array([[1, dt], [0, 1 - self._friction / self._mass]])
        B = np.array([[0], [dt / self._mass]])
        C = np.array([[1, 0]])
        D = np.array([[0]])
        
        u = np.array(self._force)
        
        # Hybrid system: zero control stops velocity
        for i, control in enumerate(u[0]):
            if control == 0:
                self._state[1][i] = 0
        
        self._state = np.dot(A, self._state) + np.dot(B, u)
        self._output = np.dot(C, self._state) + np.dot(D, u)
    
    def step(self, input) -> None:
        """Execute one dynamics step with given force input."""
        if self._name == 'integrator':
            self._force = input
            self.SS_dynamics()
        else:
            raise NotImplementedError()