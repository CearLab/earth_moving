import numpy as np
import beaverbot.ral.algorithms.module_misc as module_misc
from scipy.linalg import expm


class Controller:
    """
    Multi-purpose controller class for robot motion and navigation control.
    
    This class implements various control algorithms for robotic systems,
    with support for both simple proportional control and advanced repulsive
    potential field control. It provides flexible configuration options for
    different control strategies and environmental constraints.
    
    Supported Controllers:
    - 'P': Simple proportional controller for basic tracking
    - 'P_repulsive': Proportional controller with repulsive potential fields
                     for obstacle avoidance and navigation in complex environments
    
    Attributes:
        _name (str): Type of controller ('P' or 'P_repulsive')
        _max (float): Maximum control output magnitude
        _accuracy (float): Convergence threshold for control error
        _dimension (int): Control space dimensionality (typically 2 for 2D motion)
        _visits_reset (bool): Whether to reset visit tracking
        _Kp (float): Proportional gain parameter
        _status (str): Current controller status ('IDLE', 'ACTIVE', 'FINISHED')
        _dt (float): Time step for control updates
        _current_value (np.ndarray): Current system state/position
        _setpoint (np.ndarray): Desired target state/position
        _error (np.ndarray): Current control error
        _output (np.ndarray): Computed control signal
        
    P_repulsive specific attributes:
        _map_repulsive (bool): Enable repulsive potential mapping
        _neighbourhood_size (int): Size of neighborhood for potential calculation
        _beta_repulsive (float): Balance factor between attractive/repulsive forces
        _alpha_memory (float): Memory factor for previous control actions
        _vegetation_barrier (float): Vegetation-based barrier threshold
        _river_barrier (float): Water/river barrier threshold
        _variance (float): Control signal variance measure
        _previous_control (np.ndarray): Previous control output for smoothing
    
    Example:
        >>> # Simple P controller
        >>> controller = Controller(
        ...     controller={
        ...         'name': 'P',
        ...         'Kp': 1.0,
        ...         'max': 5.0,
        ...         'accuracy': 0.1,
        ...         'dimension': 2
        ...     }
        ... )
        >>> output = controller.step(setpoint=[10, 10], current_value=[0, 0])
        
        >>> # Repulsive potential field controller
        >>> controller = Controller(
        ...     controller={
        ...         'name': 'P_repulsive',
        ...         'Kp': 2.0,
        ...         'max': 3.0,
        ...         'accuracy': 0.2,
        ...         'dimension': 2,
        ...         'beta_repulsive': 0.3,
        ...         'neighbourhood_size': 5
        ...     }
        ... )
    """
    
    def __init__(self, **kwargs) -> None:
        
        controller = kwargs.get('controller')
        self._name = controller.get('name')
        
        self._max = controller.get('max')            
        self._accuracy = controller.get('accuracy')
        self._dimension = controller.get('dimension') 
        self._visits_reset = controller.get('visits_reset') 
        
        if self._name == 'P':            
            self._Kp = controller.get('Kp')                        
            self._Kd = controller.get('Kd')   
            self._Ki = controller.get('Ki')   
                                    
        if self._name == 'P_repulsive':
            self._Kp = controller.get('Kp')   
            self._Kd = controller.get('Kd')     
            self._Ki = controller.get('Ki')                   
            self._map_repulsive = controller.get('map_repulsive')
            self._neighbourhood_size = controller.get('neighbourhood_size')            
            self._beta_repulsive = controller.get('beta_repulsive')
            self._alpha_memory = controller.get('alpha_memory')
            self._vegetation_barrier = controller.get('vegetation_barrier')
            self._river_barrier = controller.get('river_barrier')       
            
            # attributes
            self._variance = 0
            self._previous_control = np.zeros((1, self._dimension))
            
        self._Kp_init = self._Kp
        self._Kd_init = self._Kd
        self._Ki_init = self._Ki
                        
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
        """
        Compute control error between setpoint and current value.
        
        This method calculates the error signal used by the controller,
        with different computation methods depending on the controller type.
        For simple P controllers, it computes direct error. For P_repulsive
        controllers, it computes errors for multiple positions in the
        neighborhood.
        
        Args:
            setpoint (float or array-like): Desired target value/position
            current_value (float or array-like): Current system value/position.
                For P_repulsive controllers, this can be a list of positions
                in the neighborhood.
        
        Returns:
            np.ndarray or list: Computed error signal(s)
            
        Side Effects:
            - Updates _current_value, _setpoint, and _error attributes
            
        Note:
            For P_repulsive controllers, current_value should contain multiple
            positions for neighborhood-based error computation.
        """                
        
        # error
        if self._name == 'P':            
            error = setpoint - current_value
        elif self._name == 'P_repulsive':
            error = setpoint - current_value[-1]
        else:
            raise NotImplementedError()
        
        # store info
        self._current_value = current_value
        self._setpoint = setpoint
        self._error_old = self._error        
        self._error = error
        self._error_integral += self._error
        
        self._max_scaled = self._max        
        
        return error
        
    def compute_output(self, error, neighbourhood = None) -> None:
        """
        Compute control output signal based on error and controller type.
        
        This method implements the core control logic, supporting both simple
        proportional control and advanced repulsive potential field control.
        The P_repulsive controller uses neighborhood information to balance
        attractive forces (toward setpoint) and repulsive forces (away from
        obstacles or undesirable areas).
        
        Args:
            error (np.ndarray or list): Control error signal(s)
            neighbourhood (tuple, optional): For P_repulsive controllers,
                contains (positions, values) where:
                - positions: List of [x, y] coordinates in neighborhood
                - values: Corresponding quality/cost values at each position
        
        Returns:
            np.ndarray: Computed control signal, clipped to maximum bounds
            
        Algorithm Details:
        - P controller: Simple proportional control (Kp * error)
        - P_repulsive controller:
            - Far from target: Uses potential field combining attractive and
              repulsive forces based on neighborhood values
            - Near target: Switches to simple proportional control
            - Balances forces using beta_repulsive parameter
            
        Note:
            Output is automatically clipped to [-_max, _max] range and
            reshaped to (1, dimension) format.
        """                
        
        # compute the control signal
        if self._name == 'P':
            error_d = (error - self._error_old)
            control = self._Kp * error + self._Kd * error_d + self._Ki * self._error_integral

        elif self._name == 'P_repulsive':
            # Compute the repulsive potential based on the neighborhood            
            neighbourhood_pos = neighbourhood[0]
            neighbourhood_values = neighbourhood[1]
            
            # normalize values                        
            # Rescale all values within 0 and np.linalg.norm(error)
            min_val = np.min(neighbourhood_values)
            max_val = np.max(neighbourhood_values)
            if max_val > min_val:
                neighbourhood_values = (neighbourhood_values - min_val) / (max_val - min_val) * np.linalg.norm(error)
            else:
                neighbourhood_values = np.zeros_like(neighbourhood_values)
            
            # init
            value_list = []
            
            # balance between attractive and repulsive potential            
            # far, repulsive potential
            if np.linalg.norm(error) > 4.0:
                for pos, val in zip(neighbourhood_pos, neighbourhood_values):
                    if not(pos[0] == self._current_value[-1][0] and pos[1] == self._current_value[-1][1]):
                        value_rep = val
                        value_list.append([pos[0], pos[1], value_rep])
                    else:
                        value_rep = np.inf
                        value_pos = val
                        value_list.append([pos[0], pos[1], value_rep])

                # Find the position corresponding to the minimum value in the third column
                value_array = np.array(value_list)
                # Filter out rows where the third column is inf
                finite_mask = np.isfinite(value_array[:, 2])
                finite_array = value_array[finite_mask]
                potential_array = []
                for i in range(finite_array.shape[0]):
                    potential = (value_pos - finite_array[i, 2]) * (self._current_value[-1] - finite_array[i, :2])
                    potential_array.append(potential)
            else:
                potential_array = np.array((0.0, 0.0))                
                
            # if I'm in the river, my KP increases
            # Select D4 (4-connected) neighbors of the current position
            current = self._current_value[-1]
            d4_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]
            d4_neighbors = []
            d4_values = []

            for offset in d4_offsets:
                neighbor = [current[0] + offset[0], current[1] + offset[1]]
                for pos, val in zip(neighbourhood_pos, neighbourhood_values):
                    if np.allclose(pos, neighbor):
                        d4_neighbors.append(pos)
                        d4_values.append(val)
            
            # gains
            beta = self._beta_repulsive
            Kp = self._Kp
            Kd = self._Kd
            Ki = self._Ki
            self._max_scaled = self._max

            # derivative
            error_d = (error - self._error_old)
                
            # control            
            control_attractive = Kp * error + Kd * error_d + Ki * self._error_integral
            control_repulsive = Kp * np.sum(potential_array, axis=0)
            control=  beta * control_repulsive + (1 - beta) * control_attractive

        else:
            raise NotImplementedError()
            
        _output = np.clip(control, -self._max_scaled, self._max_scaled)
        _output = _output.reshape((1, self._dimension))
        return _output
        
    def P_controller(self, setpoint: float, current_value: float) -> None:
        """
        Simple proportional controller implementation.
        
        This method implements a basic proportional controller that computes
        control output directly proportional to the error between setpoint
        and current value. Suitable for simple tracking tasks without
        environmental constraints.
        
        Args:
            setpoint (float or array-like): Desired target position/value
            current_value (float or array-like): Current system position/value
        
        Returns:
            np.ndarray: Control output signal
            
        Formula:
            output = Kp * (setpoint - current_value)
        """
        # convert values type
        setpoint = np.array(setpoint)
        current_value = np.array(current_value)
        
        # control
        error = self.compute_error(setpoint, current_value)
        output = self.compute_output(error)
        
        return output
    
    def P_repulsive_controller(self, setpoint: float, current_value: float, _neighbourhood_values: list) -> None:
        """
        Proportional controller with repulsive potential fields.
        
        This advanced controller combines proportional control with repulsive
        potential fields for navigation in complex environments. It considers
        the quality/cost of neighboring positions to avoid obstacles and
        navigate around undesirable areas while moving toward the setpoint.
        
        Args:
            setpoint (float or array-like): Desired target position
            current_value (float or array-like): Current system position
            _neighbourhood_values (list): Tuple containing:
                - [0]: List of neighboring positions [[x1,y1], [x2,y2], ...]
                - [1]: List of corresponding quality/cost values [v1, v2, ...]
        
        Returns:
            np.ndarray: Control output signal considering both attractive
                       forces (toward setpoint) and repulsive forces (away
                       from high-cost areas)
        
        Features:
        - Automatic switching between potential field and simple P control
        - Normalization of neighborhood values for consistent behavior
        - Balanced attractive/repulsive force computation
        - Obstacle avoidance through potential field navigation
        """
        # convert values type
        setpoint = np.array(setpoint)
        current_value = np.array(current_value)
        pos_values = np.array(_neighbourhood_values[0])                      
        
        # control
        error = self.compute_error(setpoint, pos_values)
        output = self.compute_output(error, _neighbourhood_values)
        
        return output
        
    def step(self, setpoint: float, current_value: float, _neighbourhood = None) -> None:
        """
        Execute one control step with automatic status management.
        
        This is the main interface method for the controller. It executes
        the appropriate control algorithm based on the controller type,
        computes the control output, and automatically manages the controller
        status based on convergence criteria.
        
        Args:
            setpoint (float or array-like): Desired target position/value
            current_value (float or array-like): Current system position/value
            _neighbourhood (list, optional): For P_repulsive controllers,
                neighborhood information for potential field computation
        
        Side Effects:
            - Updates _output with computed control signal
            - Updates _status based on convergence:
                - 'FINISHED': Error below accuracy threshold
                - 'ACTIVE': Error above threshold, control active
                - 'IDLE': Reset from finished state
        
        Controller Status States:
        - 'IDLE': Controller not actively controlling
        - 'ACTIVE': Controller actively computing control signals
        - 'FINISHED': Target reached within accuracy tolerance
        
        Note:
            The method automatically resets from 'FINISHED' to 'IDLE' state
            on the next call, allowing for new control tasks.
        """
        if self._status == 'FINISHED':
            self._status = 'IDLE'
            return
        
        if self._name == 'P':
            self._output = self.P_controller(setpoint, current_value)
            error_check = np.linalg.norm(self._error)
        elif self._name == 'P_repulsive':
            self._output = self.P_repulsive_controller(setpoint, current_value, _neighbourhood)
            error_check = np.linalg.norm(self._error)
        
        # check accuracy and update status
        if error_check <= self._accuracy:
            self._status = 'FINISHED'         
        else:
            self._status = 'ACTIVE'
            

class Dynamics:
    """
    System dynamics simulator for robotic motion modeling.
    
    This class implements various dynamic system models to simulate the
    behavior of robotic systems under control inputs. It provides state-space
    representations of common robotic dynamics with configurable parameters
    for mass, friction, and other physical properties.
    
    Supported Dynamics:
    - 'integrator': Double integrator dynamics with friction for position/velocity
                   modeling of mobile robots or manipulator joints
    
    The integrator model represents a mass-spring-damper system:
    - State: [position, velocity]
    - Input: force
    - Output: position
    - Physics: F = ma with friction forces
    
    Attributes:
        _name (str): Type of dynamics model ('integrator')
        _mass (float): System mass parameter
        _friction (float): Friction coefficient
        _status (str): Current dynamics status ('IDLE', 'ACTIVE')
        _dt (float): Time step for numerical integration
        _state (np.ndarray): Current system state [position, velocity]
        _force (list): Applied control force/input
        _output (list): System output (typically position)
    
    State-Space Representation (Integrator):
        x[k+1] = A*x[k] + B*u[k]
        y[k] = C*x[k] + D*u[k]
        
        Where:
        - A = [[1, dt], [0, 1-friction/mass]]
        - B = [[0], [dt/mass]]
        - C = [[1, 0]]
        - D = [[0]]
    
    Example:
        >>> dynamics = Dynamics(
        ...     initial_state=np.array([[0], [0]]),  # [position, velocity]
        ...     dynamics={
        ...         'name': 'integrator',
        ...         'mass': 1.0,
        ...         'friction': 0.1
        ...     }
        ... )
        >>> dynamics._dt = 0.1
        >>> dynamics.step(input=[[1.0]])  # Apply force of 1.0
        >>> print(dynamics._output)  # Get position output
    """
    
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
        """
        State-space dynamics computation for integrator model.
        
        This method implements the discrete-time state-space representation
        of a double integrator system with friction. It updates the system
        state based on the current state, applied forces, and system parameters.
        
        State-Space Model:
            x[k+1] = A*x[k] + B*u[k]
            y[k] = C*x[k] + D*u[k]
        
        Where:
        - x = [position, velocity]^T
        - u = applied_force
        - y = position (output)
        
        Matrices:
        - A = [[1, dt], [0, 1-friction/mass]]  # State transition
        - B = [[0], [dt/mass]]                 # Input matrix
        - C = [[1, 0]]                         # Output matrix
        - D = [[0]]                            # Feedthrough matrix
        
        Hybrid System Feature:
        The method implements a hybrid system behavior where zero control
        input immediately stops the velocity, simulating perfect braking
        or position holding capability.
        
        Side Effects:
            - Updates _state with new [position, velocity]
            - Updates _output with new position
            - Applies hybrid stopping logic for zero inputs
        
        Note:
            The friction term (1-friction/mass) provides velocity-dependent
            damping, while the mass term affects acceleration response to
            applied forces.
        """                                
        dt = self._dt
        
        # closed form discrete dynamics with smaller time step
        A = np.array([[1, dt], [0, 1 - self._friction/self._mass]])
        B = np.array([[0], [dt/self._mass]])
        C = np.array([[1, 0]])
        D = np.array([[0]])
        
        # control input
        u = np.array(self._force)                

        # trick to stop the state - hybrid system
        for i, control in enumerate(u[0]):
            if control == 0:
                self._state[1][i] = 0        
        
        # Integrate        
        self._state = np.dot(A, self._state) + np.dot(B, u)        
        
        # Final output computation
        self._output = np.dot(C, self._state) + np.dot(D, u)
        
    def step(self, input) -> None:
        """
        Execute one dynamics simulation step.
        
        This is the main interface method for the dynamics simulator. It
        accepts control inputs and updates the system state according to
        the configured dynamics model.
        
        Args:
            input (array-like): Control input/force to apply to the system.
                              For integrator dynamics, this represents force
                              applied to the mass.
        
        Side Effects:
            - Updates _force with the input value
            - Calls appropriate dynamics computation method
            - Updates _state and _output through dynamics computation
        
        Raises:
            NotImplementedError: If dynamics model name is not supported
        
        Supported Models:
        - 'integrator': Double integrator with friction dynamics
        
        Example:
            >>> dynamics.step([1.5])  # Apply force of 1.5
            >>> position = dynamics._output[0]  # Get current position
            >>> velocity = dynamics._state[1]   # Get current velocity
        """
        if self._name == 'integrator':
            self._force = input
            self.SS_dynamics()
        else:
            raise NotImplementedError()
        
        
    
        