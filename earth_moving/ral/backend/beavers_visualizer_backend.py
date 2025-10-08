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
        
        # generate beaversBeaversVisualizerBackend
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
        # v_normalizer = 1     
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
                                        
        ## FIG1 - LOCAL MAPS (1x3 subplots: local_map_init, local_map, local_map_visits)
        fig = plt.figure(figsize=(25, 8))
        ax1 = plt.subplot(1, 3, 1)  # Left
        ax2 = plt.subplot(1, 3, 2)  # Middle  
        ax3 = plt.subplot(1, 3, 3)  # Right
        
        # Get first agent for local map visualization
        first_agent = None
        for agent in self._schedule.agents:
            if isinstance(agent, BeaversVisualizerAgent):
                first_agent = agent
                break
        
        if first_agent is None:
            print("No agents found for local map visualization")
            return
                
        # box around the local maps        
        box_margin = 0.5
        map_width = first_agent._local_map.shape[0] if first_agent._local_map is not None else self._width
        map_height = first_agent._local_map.shape[1] if first_agent._local_map is not None else self._height
        
        box_left = plt.Rectangle((0, 0), map_width-1, map_height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        box_middle = plt.Rectangle((0, 0), map_width-1, map_height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        box_right = plt.Rectangle((0, 0), map_width-1, map_height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        ax1.add_patch(box_left)
        ax2.add_patch(box_middle)
        ax3.add_patch(box_right)

        # Get local maps from first agent
        if first_agent._local_map is not None:
            local_map = self._environment._map
            local_map_visits = self._environment._map_visits_roles
            
            # Create initial local map (use environment's initial map cropped to local map size)
            local_map_init = self._environment._initial_map[:map_width, :map_height]
            
            # Normalize maps
            local_map_init_normalized = local_map_init / v_normalizer
            local_map_normalized = local_map / v_normalizer
            
            # For visits map, use different normalization            
            max_visits = first_agent._maximum_load_init
            local_map_visits_normalized = local_map_visits / max_visits

            # Plot local_map_init on the left (ax1)
            im1 = ax1.imshow(local_map_init_normalized.transpose(), origin='lower', 
                             cmap=map_colormap, alpha=alpha_map,
                             vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
            
            # Plot current local_map in the middle (ax2)
            im2 = ax2.imshow(local_map_normalized.transpose(), origin='lower', 
                             cmap=map_colormap, alpha=alpha_map,
                             vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
            
            # Plot local_map_visits on the right (ax3)            
            im3 = ax3.imshow(local_map_visits_normalized.transpose(), origin='lower', 
                             cmap=self._color_maps._visits_colormap, alpha=0.8,
                             vmin=-1, vmax=1)
        else:
            # Fallback to environment maps if local maps not available
            map_normalized = self._environment._map_original / v_normalizer
            initial_map_normalized = self._environment._map_original / v_normalizer
            visits_normalized = self.np.zeros_like(map_normalized)
            
            im1 = ax1.imshow(initial_map_normalized.transpose(), origin='lower', 
                             cmap=map_colormap, alpha=alpha_map,
                             vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
            im2 = ax2.imshow(map_normalized.transpose(), origin='lower', 
                             cmap=map_colormap, alpha=alpha_map,
                             vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
            im3 = ax3.imshow(visits_normalized.transpose(), origin='lower', 
                             cmap=self._color_maps._visits_colormap, alpha=0.8,
                             vmin=-1, vmax=1)
        
        # Add shared colorbar for vegetation quality (for ax1 and ax2) - positioned with more spacing
        # Create space for colorbars by adjusting subplot positions
        plt.subplots_adjust(right=0.85)
        cbar_ax1 = fig.add_axes([0.86, 0.15, 0.02, 0.7])  # [left, bottom, width, height] for vegetation quality
        cbar1 = plt.colorbar(im2, cax=cbar_ax1)
        cbar1.set_label('Vegetation Quality / Elevation', rotation=270, labelpad=20)
        # Set colorbar ticks to show actual values (not normalized) - more detailed ticks        
        n_ticks = 9  # Number of ticks (including min and max)
        tick_values = self.np.linspace(vmin, vmax, n_ticks)
        cbar_ticks = tick_values / v_normalizer  # Normalize for colorbar
        cbar_labels = [f'{val:.1f}' for val in tick_values]
        cbar1.set_ticks(cbar_ticks)
        cbar1.set_ticklabels(cbar_labels)
        
        # Add separate colorbar for visits map (ax3) - positioned with more spacing from first colorbar
        cbar_ax2 = fig.add_axes([0.92, 0.15, 0.02, 0.7])  # [left, bottom, width, height] for visits - more spaced
        cbar2 = plt.colorbar(im3, cax=cbar_ax2)
        cbar2.set_label('Visit Frequency', rotation=270, labelpad=20)                               

        # Overlay agent positions (on local map - ax2)
        if plot_agents:
            for agent in self._schedule.agents:
                if isinstance(agent, BeaversVisualizerAgent):
                    # Convert global position to local map coordinates
                    local_x = agent._position[0]
                    local_y = agent._position[1]
                    
                    # plot by color based on role
                    if agent._role == 'explorer':
                        agentcolor = 'red'
                    else:
                        agentcolor = 'black'
                        
                    # Only plot if within local map bounds
                    if (0 <= local_x < map_width and 0 <= local_y < map_height):
                        ax2.plot(local_x, local_y, 
                            agent_marker, 
                            markersize=      agent_markersize, 
                            markeredgecolor= agent_markeredgecolor,
                            markerfacecolor= agentcolor,
                            markeredgewidth= agent_markeredgewidth,
                            alpha=           agent_markeralpha)                                        
                    
        for agent in self._schedule.agents:
            # add a box around home_position (on local map - ax2)
            if agent._home_base_position_store is not None:
                for home_base_position in agent._home_base_position_store:
                    local_x = home_base_position[0]
                    local_y = home_base_position[1]
                    # Only plot if within local map bounds
                    if (0 <= local_x < map_width and 0 <= local_y < map_height):
                        box = plt.Rectangle((local_x - 2, local_y - 2), 3, 3, 
                                            fill=True, edgecolor=self._color_maps._black, facecolor=self._color_maps._gray, linestyle='-', linewidth=2)
                        ax2.add_patch(box)
        
        # Configure all three axes
        titles = ["Vegetation Quality (Initial)", "Vegetation Quality (Current)", "Visits (Current)"]
        for i, (ax, title) in enumerate([(ax1, titles[0]), (ax2, titles[1]), (ax3, titles[2])]):
            ax.set_aspect('equal')
            ax.grid(False)
            
            # For local maps, use simpler coordinate system
            if i < 2:  # For vegetation quality maps (ax1, ax2)
                ax.set_xlabel('X [pixels]', fontsize=12)
                ax.set_ylabel('Y [pixels]', fontsize=12)
            else:  # For visits map (ax3)
                ax.set_xlabel('X [pixels]', fontsize=12)
                ax.set_ylabel('Y [pixels]', fontsize=12)
            
            # Set axis limits in pixel coordinates for local maps
            ax.set_xlim(0 - box_margin, map_width + box_margin)
            ax.set_ylim(0 - box_margin, map_height + box_margin)
            ax.set_title(title)
        
        # Add time information to the middle plot
        ax3.text(0.5, 1, f"DAY: {self._environment._current_day} HOUR: {self._environment._current_hour}h", 
                 fontsize=14, color=self._color_maps._black, font=fontname)                         
                                
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
            exploration_eta_values = agent._exploration_eta_store
            
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
                               alpha=0.8,
                               label='Current Load')
                
                # Plot maximum load capacity if available
                if hasattr(agent, '_maximum_load_store') and len(agent._maximum_load_store) > 0:
                    max_loads = agent._maximum_load_store
                    axes[i, 1].plot(time_steps[:len(max_loads)], max_loads,
                                   color='red',
                                   linewidth=1.5,
                                   alpha=0.7,
                                   linestyle='--',
                                   label='Maximum Load Capacity')
                
                axes[i, 1].set_xlabel('Time Steps')
                axes[i, 1].set_ylabel('Load Amount')
                axes[i, 1].set_title(f'Agent {agent_id}: Load Variation')
                axes[i, 1].grid(True, alpha=0.3)
                axes[i, 1].set_ylim(0, agent._maximum_load_init + 1)
                axes[i, 1].legend()
                
                # Third plot: Error norm over time
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
                
                # Fourth plot: Exploration eta and harvest thresholds over time
                if len(exploration_eta_values) > 0:
                    # Plot exploration eta
                    axes[i, 3].plot(time_steps[:len(exploration_eta_values)], exploration_eta_values,
                                   color=colors[i],
                                   linewidth=2,
                                   alpha=0.8,
                                   label='Exploration Eta')
                    
                    # Plot harvest thresholds if available
                    if hasattr(agent, '_harvest_threshold_store') and len(agent._harvest_threshold_store) > 0:
                        harvest_thresholds = agent._harvest_threshold_store
                        # Extract lower and upper bounds
                        lower_bounds = [th[0] for th in harvest_thresholds if len(th) >= 2]
                        upper_bounds = [th[1] for th in harvest_thresholds if len(th) >= 2]
                        
                        if len(lower_bounds) > 0 and len(upper_bounds) > 0:
                            axes[i, 3].plot(time_steps[:len(lower_bounds)], lower_bounds,
                                           color='red',
                                           linewidth=1.5,
                                           alpha=0.7,
                                           linestyle='--',
                                           label='Harvest Threshold Min')
                            axes[i, 3].plot(time_steps[:len(upper_bounds)], upper_bounds,
                                           color='orange',
                                           linewidth=1.5,
                                           alpha=0.7,
                                           linestyle='--',
                                           label='Harvest Threshold Max')
                    
                    axes[i, 3].set_xlabel('Time Steps')
                    axes[i, 3].set_ylabel('Values')
                    axes[i, 3].set_title(f'Agent {agent_id}: Exploration Eta & Harvest Thresholds')
                    axes[i, 3].grid(True, alpha=0.3)
                    axes[i, 3].legend()
                else:
                    axes[i, 3].text(0.5, 0.5, 'No exploration eta data available', 
                                   ha='center', va='center', transform=axes[i, 3].transAxes)
                    axes[i, 3].set_title(f'Agent {agent_id}: Exploration Eta & Harvest Thresholds')
                
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
                axes[i, 3].text(0.5, 0.5, 'No data available', 
                               ha='center', va='center', transform=axes[i, 3].transAxes)
                axes[i, 0].set_title(f'Agent {agent_id}: Distance from Initial Position')
                axes[i, 1].set_title(f'Agent {agent_id}: Load Variation')
                axes[i, 2].set_title(f'Agent {agent_id}: Control Error Norm')
                axes[i, 3].set_title(f'Agent {agent_id}: Exploration Eta & Harvest Thresholds')
        
        # Adjust layout
        plt.tight_layout()                
        
        if self._gui:
            plt.show()
    
    def save_environment_map(self, file_path: str) -> None:
        """
        Save the current environment map to a .npy file.
        
        This method saves the current state of the environment map (_map) to a numpy
        binary file format (.npy). This is useful for:
        - Saving simulation states for later analysis
        - Creating snapshots of environment evolution
        - Exporting maps for use in other applications
        - Backup and restoration of simulation states
        
        Args:
            file_path (str): The path where to save the .npy file. Should include
                           the .npy extension. If the directory doesn't exist,
                           it will be created automatically.
        
        Side Effects:
            - Creates the specified file with the environment map data
            - Creates parent directories if they don't exist
            
        Raises:
            FileNotFoundError: If the environment is not initialized
            PermissionError: If write permissions are insufficient
            OSError: If there are issues with file system operations
            
        Example:
            >>> visualizer.save_environment_map('/path/to/output/environment_map.npy')
            >>> visualizer.save_environment_map('output/simulation_state_t100.npy')
        
        Note:
            The saved map represents the current vegetation quality and elevation
            data as modified by agent activities during the simulation.
        """
        import os
        import numpy as np
        
        if self._environment is None:
            raise ValueError("Environment not initialized. Call generate_agents() first.")
        
        if self._environment._map is None:
            raise ValueError("Environment map not available.")
        
        # Create directory if it doesn't exist
        directory = os.path.dirname(file_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
        
        # Add .npy extension if not present
        if not file_path.endswith('.npy'):
            file_path += '.npy'
        
        # Linear rescaling to [-1, 1]
        map_data = self._environment._map.copy().astype(float)
        
        # Get min and max values
        data_min = map_data.min()
        data_max = map_data.max()
        
        # Linear rescaling: (data - data_min) / (data_max - data_min) * 2 - 1
        if data_max > data_min:
            map_data = (map_data - data_min) / (data_max - data_min) * 2 - 1
            normalization_info = f"Linear rescaling [{data_min:.3f}, {data_max:.3f}] -> [-1.0, 1.0]"
        else:
            # All values are the same
            map_data = map_data * 0  # Set all values to 0
            normalization_info = f"All values are the same ({data_min:.3f}) -> 0.0"                
        
        # Apply inverse coordinate transformation to match original DEM format
        # Environment format -> DEM format: reverse the transformations applied in load_map_from_npy
        # Original: flipud() + rot90(-1) 
        # Inverse: rot90(1) + flipud()
        map_to_save = np.rot90(map_data, 1)  # Rotate 90° clockwise (inverse of -90°)
        map_to_save = np.flipud(map_to_save)  # Flip vertically (inverse of flipud)
        
        # Save the normalized and transformed map
        np.save(file_path, map_to_save)
        
        if self._print:
            print(f"Environment map saved to: {file_path}")
            print(f"Original map shape (environment format): {self._environment._map.shape}")
            print(f"Saved map shape (DEM format): {map_to_save.shape}")
            print(f"Original value range: [{self._environment._map.min():.3f}, {self._environment._map.max():.3f}]")
            print(f"Saved value range: [{map_to_save.min():.3f}, {map_to_save.max():.3f}]")
            print(f"{normalization_info}")
            print(f"Applied inverse coordinate transformation for DEM compatibility")
        

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
            'strength': self.model._environment._flow_strength,
            'visits': self.model._environment._map_visits            
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
        map_visits = self._map_visits.copy()
        map_visits_roles = self._map_visits_roles.copy()
        map = self._map.copy()
        home_base_position_store = []
        for agents in self.model._schedule.agents:
            if isinstance(agents, BeaversVisualizerAgent):
                if agents._local_map_visits is not None:                                        
                    map_visits[agents._position[0], agents._position[1]] = agents._local_map_visits[agents._position[0], agents._position[1]]
                    map[agents._position[0], agents._position[1]] = agents._map_quality_measure_position
                    if agents._role == 'explorer':
                        map_visits_roles[agents._position[0], agents._position[1]] = map_visits[agents._position[0], agents._position[1]]
                    else:
                        map_visits_roles[agents._position[0], agents._position[1]] = -map_visits[agents._position[0], agents._position[1]]
                if agents._home_base_position_store is not None:
                    for pos in agents._home_base_position_store:
                        home_base_position_store.append(pos)
        map_visits = map_visits * self._visits_reset
        map_visits_roles = map_visits_roles * self._visits_reset
        misc = {'map_visits_roles': map_visits_roles}
        # Remove duplicates in home_base_position_store
        home_base_position_store = list(set(tuple(pos) for pos in self.np.array(home_base_position_store)))

        # step the environment
        self.step_environment(self._timedelta, map, map_visits, home_base_position_store, self._grass_growth_interval, misc)