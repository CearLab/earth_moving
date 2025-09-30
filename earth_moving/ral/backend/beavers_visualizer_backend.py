# general imports
import matplotlib.pyplot as plt
from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid

# backend imports
from earth_moving.ral.backend.base_backend import BaseBackend
from earth_moving.ral.robot.robot_beavers_backend import BeaversRobotBackend
from earth_moving.ral.environment.environment_beavers_backend import BeaversEnvironmentBackend

# module imports
from earth_moving.ral.backend.modules.module_colors import ColorMaps
import earth_moving.ral.algorithms.module_misc as module_misc


class BeaversVisualizerBackend(BaseBackend, Model):
    """
    Visualization backend for the Beavers earth-moving simulation system.
    
    This class extends both BaseBackend and Mesa's Model to provide a complete
    visualization framework for multi-agent earth-moving simulations. It manages
    the visual representation of the environment, agents, and their interactions
    over time.
    
    The class handles:
    - Agent scheduling and synchronization
    - Environment visualization with heatmaps
    - Real-time plotting of agent positions and states
    - Time management and simulation stepping
    - Color mapping and visual styling
    
    Attributes:
        _kwargs (dict): Configuration parameters from simulation setup
        _timedelta (float): Time step increment for simulation
        _schedule_policy (str): Agent scheduling policy ('sequential' or 'random')
        _gui (bool): Whether to display GUI visualization
        _N_agents (int): Number of agents in the simulation
        _print (bool): Whether to enable print statements
        _fig (matplotlib.figure.Figure): Current matplotlib figure
        _schedule (RandomActivation): Mesa scheduler for agent execution
        _current_time (float): Current simulation time
        _color_maps (ColorMaps): Color mapping utilities for visualization
        _environment (EnvironmentVisualizerAgent): Environment agent instance
        _width (int): Environment width in grid units
        _height (int): Environment height in grid units
        _grid (MultiGrid): Mesa grid for spatial agent management
    
    Example:
        >>> visualizer = BeaversVisualizerBackend(
        ...     simulation={
        ...         'timedelta': 0.1,
        ...         'schedule_policy': 'sequential',
        ...         'gui': True,
        ...         'number_of_agents': 5,
        ...         'print': True
        ...     }
        ... )
        >>> visualizer.generate_agents()
        >>> visualizer.step()
        >>> visualizer.plot_environment_with_heatmap()
    """
    
    def __init__(self, **kwargs) -> None:            
                            
        # attributes from simulation
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._schedule_policy = simulation.get('schedule_policy')
        self._gui = simulation.get('gui')
        self._N_agents = simulation.get('number_of_agents')
        self._print = simulation.get('print')
        self._fig = None
                
        # mesa init
        self._schedule = RandomActivation(self)  # Alternative: StagedActivation, SimultaneousActivation, or BaseScheduler
        
        # other attributes
        self._current_time = 0
        
        # colormaps
        self._color_maps = ColorMaps()                    
        
    def generate_agents(self, **kwargs) -> None:
        """
        Generate and initialize environment and agent instances for the simulation.
        
        This method creates the environment agent and populates the simulation with
        beaver agents according to the specified configuration. The environment is
        created first to establish spatial boundaries, followed by beaver agents
        that are placed on the grid and added to the scheduler.
        
        Args:
            **kwargs: Additional keyword arguments passed to agent constructors.
                     Typically includes configuration for agent behavior, sensors,
                     and initial conditions.
        
        Side Effects:
            - Creates _environment (EnvironmentVisualizerAgent)
            - Initializes _grid (MultiGrid) with environment dimensions
            - Creates and places N beaver agents on the grid
            - Adds all beaver agents to the scheduler
            
        Note:
            The environment agent is not added to the scheduler as it requires
            special handling and always steps first in the simulation cycle.
        """
        # generate environment
        self._environment = EnvironmentVisualizerAgent(self._N_agents, self, **kwargs) #! this is not added to the scheduler, it's always the first            
        # grid init
        self._width = self._environment._width
        self._height = self._environment._height                 
        self._grid = MultiGrid(self._width, self._height, torus=False) #! why are the dimension set in the opposite order?        
        
        # generate beavers
        for i in range(self._N_agents):            
            agent = BeaversVisualizerAgent(i, self, **kwargs)
            self._grid.place_agent(agent, (agent._position[0], agent._position[1]))
            self._schedule.add(agent)
        
    def step(self) -> None:
        """
        Execute one simulation time step for all agents and environment.
        
        This method advances the simulation by one time increment, updating
        the global time and coordinating the execution of all simulation
        components. The environment always steps first to update global
        conditions, followed by agent steps according to the configured
        scheduling policy.
        
        The method supports two scheduling policies:
        - 'sequential': Agents step in deterministic order by unique_id
        - 'random': Agents step in randomized order each time step
        
        Side Effects:
            - Increments _current_time by _timedelta
            - Steps the environment agent to update global state
            - Steps all beaver agents according to schedule policy
            
        Raises:
            ValueError: If an invalid schedule_policy is configured
            
        Note:
            All agents step synchronously - the current time waits for all
            agents to complete their step before advancing to the next cycle.
        """
        self._current_time += self._timedelta #!! the agents step synchronously (_current_time wait for all agents to step)
        
        # always step the environment first
        self._environment.step()
        
        if self._schedule_policy == 'sequential':
            # Sort agents by their unique_id
            sorted_agents = sorted(self._schedule.agents, key=lambda agent: agent.unique_id)
            for agent in sorted_agents:                
                agent.step()
        elif self._schedule_policy == 'random':
            self._schedule.step()
        else:
            raise ValueError("Invalid schedule policy.")
        
    def plot_environment_with_heatmap(self, plot_agents=True) -> None:
        """
        Generate and display a comprehensive visualization of the simulation environment.
        
        This method creates a detailed matplotlib visualization showing:
        - Environment heatmap with vegetation quality/elevation data
        - Agent positions with customizable markers (optional)
        - Home base locations marked with gray boxes
        - Time and day information overlay
        - Proper geographic coordinate system (if available)
        - Color-coded visualization based on time of day
        
        Args:
            plot_agents (bool): Whether to plot agent positions on the map (default: True)
        
        The visualization adapts its color scheme based on the time of day:
        - Day mode: Standard colors with high visibility
        - Night mode: Darker palette for night simulation
        
        Features:
        - Normalized color mapping for consistent visualization
        - Geographic coordinate labels when available
        - Agent state indicators (position, energy, etc.)
        - Home base highlighting
        - Customizable color schemes through ColorMaps
        
        Side Effects:
            - Creates and stores matplotlib figure in _fig attribute
            - Displays GUI if _gui is enabled
            - Updates visualization with current simulation state
            
        Note:
            The method includes commented code for additional visualizations
            such as agent local maps and detailed status panels that can be
            enabled for debugging or extended analysis.
        """
        
        # misc
        fontname = 'monospace'
        
        # map max and min
        vmin = -self._environment._streams_width
        vmax = self._environment._vegetation_quality_range[1]        
        
        # always same color (regardless river)
        v_normalizer = vmax              
        # different colors
        # v_normalizer = vmax - vmin
        
        # colors 
        if self._environment._time_of_day == 'day':
            # agents
            agent_marker =          self._color_maps._agent_marker
            agent_markersize =      self._color_maps._agent_markersize_small
            agent_markerfacecolor = self._color_maps._agent_markerfacecolor
            agent_markeredgecolor = self._color_maps._agent_markeredgecolor
            agent_markeredgewidth = self._color_maps._agent_markeredgewidth
            agent_markeralpha =     self._color_maps._agent_markeralpha
            
            # map
            map_colormap = self._color_maps._bluebrowngreen_colormap
            alpha_map = self._color_maps._whiteblack_colormap_alpha
        else:
            # agents
            agent_marker =          self._color_maps._agent_marker_night
            agent_markersize =      self._color_maps._agent_markersize_small_night
            agent_markerfacecolor = self._color_maps._agent_markerfacecolor_night
            agent_markeredgecolor = self._color_maps._agent_markeredgecolor_night
            agent_markeredgewidth = self._color_maps._agent_markeredgewidth_night
            agent_markeralpha =     self._color_maps._agent_markeralpha_night
            
            # map
            map_colormap = self._color_maps._bluebrowngreen_colormap
            alpha_map = self._color_maps._whiteblack_colormap_alpha
            
        # vegetation marker
        vegetation_marker = self._color_maps._vegetation_marker
        vegetation_markersize = self._color_maps._vegetation_markersize
        #! vegetation_markerfacecolor defined in the ax1.plot (it's a colormap)
        vegetation_markeredgecolor = self._color_maps._vegetation_markeredgecolor
        vegetation_markeredgewidth = self._color_maps._vegetation_markeredgewidth
        vegetation_markeralpha = self._color_maps._vegetation_markeralpha
        
        # battery marker
        battery_marker = self._color_maps._battery_marker
        battery_markersize = self._color_maps._battery_markersize
        #! battery_markerfacecolor defined in the ax1.plot (it's a colormap)
        battery_markeredgecolor = self._color_maps._battery_markeredgecolor
        battery_markeredgewidth = self._color_maps._battery_markeredgewidth
        battery_markeralpha = self._color_maps._battery_markeralpha
                                        
        ## FIG1 - ENVIRONMENT
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
                
        # box around the environment        
        box_margin = 0.5
        box_left = plt.Rectangle((0, 0), self._width-1, self._height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        box_right = plt.Rectangle((0, 0), self._width-1, self._height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        ax1.add_patch(box_left)
        ax2.add_patch(box_right)

        # Normalize maps
        map_normalized = self._environment._map / v_normalizer
        initial_map_normalized = self._environment._initial_map / v_normalizer

        # Plot initial map on the left (ax1)
        im1 = ax1.imshow(initial_map_normalized.transpose(), origin='lower', 
                         cmap=map_colormap, alpha=alpha_map,
                         vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
        
        # Plot current map on the right (ax2) with agents and overlays
        im2 = ax2.imshow(map_normalized.transpose(), origin='lower', 
                         cmap=map_colormap, alpha=alpha_map,
                         vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
        
        # Add shared colorbar that doesn't affect subplot sizes
        # Create space for colorbar by adjusting subplot positions
        plt.subplots_adjust(right=0.85)
        cbar_ax = fig.add_axes([0.87, 0.15, 0.03, 0.7])  # [left, bottom, width, height]
        cbar = plt.colorbar(im2, cax=cbar_ax)
        cbar.set_label('Vegetation Quality / Elevation', rotation=270, labelpad=20)
        # Set colorbar ticks to show actual values (not normalized)
        cbar_ticks = [vmin/v_normalizer, 0, vmax/v_normalizer]
        cbar_labels = [f'{vmin:.1f}', '0.0', f'{vmax:.1f}']
        cbar.set_ticks(cbar_ticks)
        cbar.set_ticklabels(cbar_labels)                                

        # Overlay agent positions (only on current map - ax2)
        if plot_agents:
            for agent in self._schedule.agents:
                if isinstance(agent, BeaversVisualizerAgent):
                    # Use pixel coordinates directly (simple approach)
                    ax2.plot(agent._position[0], agent._position[1], 
                        agent_marker, 
                        markersize=      agent_markersize, 
                        markeredgecolor= agent_markeredgecolor,
                        markerfacecolor= self._color_maps._black,
                        markeredgewidth= agent_markeredgewidth,
                        alpha=           agent_markeralpha)                                        
                    
        for agent in self._schedule.agents:
            # add a box around home_position (only on current map - ax2)
            if agent._home_base_position_store is not None:
                for home_base_position in agent._home_base_position_store:
                    # Use pixel coordinates (simple approach)
                    box = plt.Rectangle((home_base_position[0] - 2, home_base_position[1] - 2), 3, 3, 
                                        fill=True, edgecolor=self._color_maps._black, facecolor=self._color_maps._gray, linestyle='-', linewidth=2)
                    ax2.add_patch(box)
        
        # Configure both axes
        for ax, title in [(ax1, "Initial Environment"), (ax2, "Current Environment")]:
            ax.set_aspect('equal')
            ax.grid(False)
            
            # Set custom tick labels with latitude/longitude if available
            if hasattr(self._environment, 'x_axis') and hasattr(self._environment, 'y_axis'):
                # Show geographic coordinate labels
                ax.set_xlabel('X [m]', fontsize=12)
                ax.set_ylabel('Y [m]', fontsize=12)

                # Create custom tick positions and labels
                # Sample 6 points across each axis for reasonable tick spacing
                n_ticks = 6
                x_tick_positions = self.np.linspace(0, len(self._environment.x_axis)-1, n_ticks, dtype=int)
                y_tick_positions = self.np.linspace(0, len(self._environment.y_axis)-1, n_ticks, dtype=int)

                # Get corresponding geographic coordinates
                x_tick_labels = [f'{self._environment.x_axis[pos]:.4f}' for pos in x_tick_positions]
                y_tick_labels = [f'{self._environment.y_axis[pos]:.4f}' for pos in y_tick_positions]
                
                # Set the ticks
                ax.set_xticks(x_tick_positions)
                ax.set_xticklabels(x_tick_labels)
                ax.set_yticks(y_tick_positions)
                ax.set_yticklabels(y_tick_labels)
            else:
                ax.set_axis_off()
            
            # Set axis limits in pixel coordinates
            ax.set_xlim(0 - box_margin, self._width + box_margin)
            ax.set_ylim(0 - box_margin, self._height + box_margin)
            ax.set_title(title)
        
        # Add time information to the right plot
        ax2.text(0.5, 1, f"DAY: {self._environment._current_day} HOUR: {self._environment._current_hour}h", 
                 fontsize=14, color=self._color_maps._white, font=fontname)                        
                                
        self._fig = fig
        if self._gui:
            plt.show()    
    
    def plot_simulation_recap(self) -> None:
        """
        Generate agent-specific analysis plots showing distance, load, error, and exploration variations over time.
        
        This method creates a comprehensive N×4 subplot visualization where N is the number
        of agents. Each row represents one agent with four metrics:
        - Column 1: Distance from initial position over time
        - Column 2: Load variation over time
        - Column 3: Control error norm over time
        - Column 4: Exploration eta parameter over time
        
        The visualization helps analyze individual agent behavior patterns including:
        - Exploration range and movement patterns
        - Return-to-base behavior
        - Load collection and storage cycles
        - Agent-specific activity levels
        - Control system performance and error evolution
        
        Layout:
        - N×4 subplots where N = number of agents
        - Row i corresponds to Agent i
        - Column 1: Distance from initial position vs time
        - Column 2: Load vs time
        - Column 3: Control error norm vs time
        - Column 4: Exploration eta parameter vs time
        
        Side Effects:
            - Creates and displays matplotlib figure with N×4 subplots
            - Shows GUI if _gui is enabled
        
        Notes:
            This method should be called after simulation completion to analyze
            the complete behavioral patterns. Each agent gets its own row with
            distance, load, and error analysis side by side.
        """
        
        # Get number of agents
        agent_list = [agent for agent in self._schedule.agents if isinstance(agent, BeaversVisualizerAgent)]
        n_agents = len(agent_list)
        
        if n_agents == 0:
            print("No agents found for plotting")
            return
        
        # Create N×4 subplot grid (distance, load, error norm, exploration eta)
        fig, axes = plt.subplots(n_agents, 4, figsize=(25, 4 * n_agents))
        
        # Handle case where there's only one agent (axes won't be 2D)
        if n_agents == 1:
            axes = axes.reshape(1, -1)
        
        # Color palette for consistency
        colors = ['black'] * n_agents
        
        # Process each agent
        for i, agent in enumerate(agent_list):
            agent_id = agent.unique_id
            positions = agent._position_store
            loads = agent._load_store
            errors = agent._error_store
            
            if len(positions) > 0:
                # Calculate distances from initial position
                initial_pos = positions[0] if len(positions) > 0 else [0, 0]
                distances = []
                river_times = []
                river_distances = []
                
                for t, pos in enumerate(positions):
                    distance = self.np.sqrt((pos[0] - initial_pos[0])**2 + (pos[1] - initial_pos[1])**2)
                    distances.append(distance)
                    
                    # Check if position is in river (map value < -2)
                    if hasattr(self, '_environment') and self._environment._map is not None:
                        x, y = int(pos[0]), int(pos[1])
                        # Ensure coordinates are within map bounds
                        if (0 <= x < self._environment._map.shape[0] and 
                            0 <= y < self._environment._map.shape[1]):
                            if self._environment._map[x, y] < -2:
                                river_times.append(t)
                                river_distances.append(distance)
                
                # Calculate error norms
                error_norms = []
                for error in errors:
                    if hasattr(error, '__len__') and len(error) > 0:  # Check if error is array-like
                        error_norm = self.np.linalg.norm(error)
                    else:
                        error_norm = abs(error) if error is not None else 0.0
                    error_norms.append(error_norm)
                
                # Create time array
                time_steps = list(range(len(positions)))
                
                # Left plot: Distance from initial position over time
                axes[i, 0].plot(time_steps, distances, 
                               color=colors[i], 
                               linewidth=2,
                               alpha=0.8)
                
                # Add markers for river positions
                if len(river_times) > 0:
                    axes[i, 0].scatter(river_times, river_distances,
                                     color='blue',
                                     marker='o',
                                     s=50,
                                     alpha=0.8,
                                     edgecolors='darkblue',
                                     linewidth=1,
                                     label='In River')
                    axes[i, 0].legend()
                
                axes[i, 0].set_xlabel('Time Steps')
                axes[i, 0].set_ylabel('Distance from Start')
                axes[i, 0].set_title(f'Agent {agent_id}: Distance from Initial Position')
                axes[i, 0].grid(True, alpha=0.3)
                
                # Middle plot: Load over time
                axes[i, 1].plot(time_steps, loads,
                               color=colors[i],
                               linewidth=2,
                               alpha=0.8)
                axes[i, 1].set_xlabel('Time Steps')
                axes[i, 1].set_ylabel('Load Amount')
                axes[i, 1].set_title(f'Agent {agent_id}: Load Variation')
                axes[i, 1].grid(True, alpha=0.3)
                axes[i, 1].set_ylim(0, agent._maximum_load + 1)
                
                # Right plot: Error norm over time
                if len(error_norms) > 0:
                    axes[i, 2].plot(time_steps[:len(error_norms)], error_norms,
                                   color=colors[i],
                                   linewidth=2,
                                   alpha=0.8)
                    axes[i, 2].set_xlabel('Time Steps')
                    axes[i, 2].set_ylabel('Error Norm')
                    axes[i, 2].set_title(f'Agent {agent_id}: Control Error Norm')
                    axes[i, 2].grid(True, alpha=0.3)
                else:
                    axes[i, 2].text(0.5, 0.5, 'No error data available', 
                                   ha='center', va='center', transform=axes[i, 2].transAxes)
                    axes[i, 2].set_title(f'Agent {agent_id}: Control Error Norm')
                
                # Add horizontal line at y=0 for distance plot
                axes[i, 0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
                
            else:
                # Handle case with no data
                axes[i, 0].text(0.5, 0.5, 'No data available', 
                               ha='center', va='center', transform=axes[i, 0].transAxes)
                axes[i, 1].text(0.5, 0.5, 'No data available', 
                               ha='center', va='center', transform=axes[i, 1].transAxes)
                axes[i, 2].text(0.5, 0.5, 'No data available', 
                               ha='center', va='center', transform=axes[i, 2].transAxes)
                axes[i, 0].set_title(f'Agent {agent_id}: Distance from Initial Position')
                axes[i, 1].set_title(f'Agent {agent_id}: Load Variation')
                axes[i, 2].set_title(f'Agent {agent_id}: Control Error Norm')
        
        # Adjust layout
        plt.tight_layout()                
        
        if self._gui:
            plt.show()
        

class BeaversVisualizerAgent(BeaversRobotBackend, Agent):
    """
    Visualization-enabled beaver agent for earth-moving simulations.
    
    This class extends both BeaversRobotBackend and Mesa's Agent to provide
    a complete agent implementation for visualization-based simulations. It
    bridges the robot backend functionality with the Mesa agent-based modeling
    framework, enabling sophisticated multi-agent earth-moving behaviors.
    
    The agent handles:
    - Robot behavior and task execution
    - Environmental sensing and mapping
    - Integration with Mesa's scheduling system
    - Real-time data exchange with environment
    - Visualization state management
    
    Attributes:
        _timedelta (float): Time step from model for synchronization
        
    Inherits from BeaversRobotBackend:
        - All robot behavior attributes (_position, _energy, _load, etc.)
        - Task management and motion control
        - Environmental sensing capabilities
        - Local mapping and navigation
        
    Inherits from Mesa Agent:
        - unique_id: Unique identifier for agent tracking
        - model: Reference to the simulation model
        
    Example:
        >>> agent = BeaversVisualizerAgent(
        ...     unique_id=0,
        ...     model=simulation_model,
        ...     initial_position=[10, 10],
        ...     energy=100.0
        ... )
        >>> agent.step()  # Execute one simulation step
    """
    
    def __init__(self, unique_id, model, **kwargs) -> None:        
        BeaversRobotBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_robot(**kwargs)   
        
        # this section bridges the agent.step with the required fields from the environment/engine
        self._timedelta = model._timedelta
                    
    def step(self) -> None:
        """
        Execute one simulation step for the beaver agent.
        
        This method coordinates the agent's interaction with the environment
        and executes one step of the beaver's behavior. It handles data
        exchange with the environment/engine, performs environmental sensing,
        and updates both agent and environment states.
        
        The method performs the following operations:
        1. Retrieves current environment data (time, vegetation quality)
        2. Performs environmental measurements based on measurement mode
        3. Updates agent's environmental awareness and limits
        4. Executes beaver behavior step with current conditions
        5. Updates environment with agent's modifications (if enabled)
        
        Key data exchanges:
        - Gets time_of_day from environment
        - Performs vegetation/elevation measurements at current position
        - Receives environment boundaries and quality ranges
        - Updates environment map with measured quality data
        
        Side Effects:
            - Updates agent internal state through step_beaver()
            - May modify environment map if _map_quality_update is True
            - Synchronizes agent timing with model timedelta
            
        Note:
            The agent can only update the environment at its current position,
            reflecting the realistic constraint that agents can only modify
            their immediate surroundings.
        """        
        # get data from engine/environment (can't pass input to _schedule.step() method)
        dt = self._timedelta
        time_of_day = self.model._environment._time_of_day
        measure_positions, measure_values = module_misc.measure(self.model._environment._map, self._position, self._measurement_mode)
        map_quality = [measure_positions, measure_values]
                
        if self._measurement_mode is 'full_map':
            limits = module_misc.get_map_limits(self.model._environment._map)
        else:
            if self._local_map is not None:
                limits = module_misc.get_map_limits(self._local_map)
            else:
                limits = [[0, self._position[0] - 1], [0, self._position[1] - 1]]
        
        # link the vegetation quality from environment to the agent
        self._vegetation_quality_range = self.model._environment._vegetation_quality_range
        self._range_x = self.model._environment._width
        self._range_y = self.model._environment._height
        self._home_base_position_store = self.model._environment._home_base_position_store
        
        # link flow information from environment to the agent
        misc = {
            'direction': self.model._environment._flow_direction,
            'strength': self.model._environment._flow_strength
        }
        
        # step the agent
        self.step_beaver(dt, 
                         time_of_day,
                         map_quality, 
                         limits,
                         misc)
        
        #! here we update the environment with the agent's actions 
        #! note that we only update the current position because it's the only the beaver can actually change
        self.model._environment._map_original[self._position[0], self._position[1]] = self._map_quality_measure_position

class EnvironmentVisualizerAgent(BeaversEnvironmentBackend, Agent):
    """
    Environment management agent for visualization-based simulations.
    
    This class extends both BeaversEnvironmentBackend and Mesa's Agent to
    provide comprehensive environment management within the agent-based
    modeling framework. It handles all environmental dynamics including
    vegetation growth, terrain changes, and global state management.
    
    The environment agent manages:
    - Global environmental state and dynamics
    - Vegetation growth and quality tracking
    - Agent visit tracking and aggregation
    - Home base position management
    - Time-based environmental changes
    - Integration with Mesa's agent system
    
    Key responsibilities:
    - Aggregate agent activities across the environment
    - Update vegetation and terrain based on agent interactions
    - Manage global environmental parameters
    - Coordinate time-based environmental processes
    - Maintain environmental boundaries and constraints
    
    Attributes:
        _timedelta (float): Time step from model for synchronization
        
    Inherits from BeaversEnvironmentBackend:
        - Environmental state management (_map, _vegetation_quality, etc.)
        - Vegetation growth and dynamics
        - Terrain and elevation data
        - Environmental boundaries and limits
        
    Inherits from Mesa Agent:
        - unique_id: Environment identifier
        - model: Reference to the simulation model
        
    Example:
        >>> env_agent = EnvironmentVisualizerAgent(
        ...     unique_id=0,
        ...     model=simulation_model,
        ...     width=100,
        ...     height=100,
        ...     vegetation_quality_range=[0, 10]
        ... )
        >>> env_agent.step()  # Update environmental state
    """
    
    def __init__(self, unique_id, model, **kwargs) -> None:                
        BeaversEnvironmentBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_environment(**kwargs)      
        self._timedelta = model._timedelta       
                    
    def step(self) -> None:
        """
        Execute one environmental update step.
        
        This method performs a comprehensive update of the environmental state
        by aggregating agent activities and updating global environmental
        dynamics. It collects data from all agents and applies environmental
        processes such as vegetation growth and terrain changes.
        
        The method performs the following operations:
        1. Aggregates visit maps from all beaver agents
        2. Collects home base positions from all agents
        3. Removes duplicate home base positions
        4. Calculates grass growth intervals based on agent parameters
        5. Updates environmental state with aggregated data
        
        Key aggregations:
        - Sums local visit maps from all agents to create global visit tracking
        - Combines home base positions from all agents
        - Coordinates vegetation growth based on agent harvest thresholds
        
        Side Effects:
            - Updates global environmental state through step_environment()
            - Modifies vegetation quality and terrain based on agent activities
            - Updates home base position tracking
            - Applies time-based environmental processes
            
        Note:
            The environment steps after all agents have provided their local
            updates, ensuring consistent global state management across the
            simulation.
        """     
        
        # get the map_visits of all agents and sum them
        map_visits = self.np.zeros((self._width, self._height))
        home_base_position_store = []
        for agents in self.model._schedule.agents:
            if isinstance(agents, BeaversVisualizerAgent):
                if agents._local_map_visits is not None:
                    x_end = agents._local_map_visits.shape[0]
                    y_end = agents._local_map_visits.shape[1]
                    map_visits[:x_end, :y_end] += agents._local_map_visits
                if agents._home_base_position_store is not None:
                    for pos in agents._home_base_position_store:
                        home_base_position_store.append(pos)
        # Remove duplicates in home_base_position_store        
        home_base_position_store = list(set(tuple(pos) for pos in self.np.array(home_base_position_store)))

        # grass growth interval        
        grass_growth_interval = [agents._harvest_threshold[0], agents._harvest_threshold[1]]
        
        # step the environment
        self.step_environment(self._timedelta, map_visits, home_base_position_store, grass_growth_interval)        