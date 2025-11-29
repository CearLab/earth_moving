# general imports
import matplotlib.pyplot as plt
from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid

# backend imports
from beaverbot.ral.backend.base_backend import BaseBackend
from beaverbot.ral.robot.robot_beavers_backend import BeaversRobotBackend
from beaverbot.ral.environment.environment_beavers_backend import BeaversEnvironmentBackend

# module imports
from beaverbot.ral.backend.modules.module_colors import ColorMaps
import beaverbot.ral.algorithms.module_misc as module_misc


class BeaversVisualizerBackend(BaseBackend, Model):
    """Visualization backend for Beavers earth-moving simulation."""
    
    def __init__(self, **kwargs) -> None:
        # Simulation parameters
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._schedule_policy = simulation.get('schedule_policy')
        self._gui = simulation.get('gui')
        self._N_agents = simulation.get('number_of_agents')
        self._print = simulation.get('print')
        self._fig = None
        
        # Mesa scheduler
        self._schedule = RandomActivation(self)
        self._current_time = 0
        self._color_maps = ColorMaps()
        
    def generate_agents(self, **kwargs) -> None:
        """Generate environment and agent instances."""
        # Generate environment (not added to scheduler, always steps first)
        self._environment = EnvironmentVisualizerAgent(self._N_agents, self, **kwargs)
        
        # Initialize grid
        self._width = self._environment._width
        self._height = self._environment._height
        self._grid = MultiGrid(self._width, self._height, torus=False)
        
        # Generate beaver agents
        for i in range(self._N_agents):
            agent = BeaversVisualizerAgent(i, self, **kwargs)
            self._grid.place_agent(agent, (agent._position[0], agent._position[1]))
            self._schedule.add(agent)
    
    def step(self) -> None:
        """Execute one simulation time step: update time, step environment, then agents."""
        self._current_time += self._timedelta
        
        # Step environment first
        self._environment.step()
        
        # Step agents according to schedule policy
        if self._schedule_policy == 'sequential':
            sorted_agents = sorted(self._schedule.agents, key=lambda agent: agent.unique_id)
            for agent in sorted_agents:
                agent.step()
        elif self._schedule_policy == 'random':
            self._schedule.step()
        else:
            raise ValueError("Invalid schedule policy.")
    
    def plot_environment_with_heatmap(self, plot_agents=True) -> None:
        """Plot environment heatmap with agent positions, home bases, and time info."""
        # Get first agent
        first_agent = next((a for a in self._schedule.agents if isinstance(a, BeaversVisualizerAgent)), None)
        if not first_agent:
            print("No agents found for local map visualization")
            return
        
        # Map params
        is_day = self._environment._time_of_day == 'day'
        cm = self._color_maps
        vmax = self._environment._vegetation_quality_range[1]
        map_width = first_agent._local_map.shape[0] if first_agent._local_map is not None else self._width
        map_height = first_agent._local_map.shape[1] if first_agent._local_map is not None else self._height
        
        # Create figure
        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(25, 8))
        
        # Add boxes
        for ax in (ax1, ax2, ax3):
            ax.add_patch(plt.Rectangle((0, 0), map_width, map_height, fill=False, edgecolor='black', linestyle='-', linewidth=2))

        # Get and normalize maps
        if first_agent._local_map is not None:
            local_map_init = self._environment._initial_map[:map_width, :map_height] / vmax
            local_map_norm = self._environment._map / vmax
            local_map_visits_norm = self._environment._map_visits_roles / self.np.max(self.np.abs(self._environment._map_visits_roles) + 1e-5)
            vmin = -self._environment._streams_width
            
            im1 = ax1.imshow(local_map_init.transpose(), origin='lower', cmap=cm._bluebrowngreen_colormap, 
                           alpha=cm._whiteblack_colormap_alpha, vmin=vmin/vmax, vmax=1)
            im2 = ax2.imshow(local_map_norm.transpose(), origin='lower', cmap=cm._bluebrowngreen_colormap, 
                           alpha=cm._whiteblack_colormap_alpha, vmin=vmin/vmax, vmax=1)
            im3 = ax3.imshow(local_map_visits_norm.transpose(), origin='lower', cmap=cm._visits_colormap, alpha=1, vmin=-1, vmax=1)
            
            # Overlay negative values
            neg_mask = local_map_norm < 0
            if self.np.any(neg_mask):
                ax3.imshow(self.np.where(neg_mask, local_map_norm, self.np.nan).transpose(), origin='lower', 
                         cmap='gray', alpha=0.1, vmin=vmin/vmax, vmax=0)
        else:
            # Fallback
            vmin = -self._environment._streams_width
            map_norm = self._environment._map_original / vmax
            im1 = ax1.imshow(map_norm.transpose(), origin='lower', cmap=cm._bluebrowngreen_colormap, 
                           alpha=cm._whiteblack_colormap_alpha, vmin=vmin/vmax, vmax=1)
            im2 = ax2.imshow(map_norm.transpose(), origin='lower', cmap=cm._bluebrowngreen_colormap, 
                           alpha=cm._whiteblack_colormap_alpha, vmin=vmin/vmax, vmax=1)
            im3 = ax3.imshow(self.np.zeros_like(map_norm).transpose(), origin='lower', cmap=cm._visits_colormap, alpha=1, vmin=-1, vmax=1)
            
            neg_mask = map_norm < 0
            if self.np.any(neg_mask):
                ax3.imshow(self.np.where(neg_mask, map_norm, self.np.nan).transpose(), origin='lower', 
                         cmap='gray', alpha=0.1, vmin=vmin/vmax, vmax=0)
        
        # Colorbars
        plt.subplots_adjust(right=0.85)
        cbar1 = plt.colorbar(im2, cax=fig.add_axes([0.86, 0.15, 0.02, 0.7]))
        cbar1.set_label('Vegetation Quality / Elevation', rotation=270, labelpad=20)
        tick_vals = self.np.linspace(vmin, vmax, 9)
        cbar1.set_ticks(tick_vals / vmax)
        cbar1.set_ticklabels([f'{v:.1f}' for v in tick_vals])
        cbar2 = plt.colorbar(im3, cax=fig.add_axes([0.92, 0.15, 0.02, 0.7]))
        cbar2.set_label('Visit Frequency', rotation=270, labelpad=20)

        # Plot agents
        if plot_agents:
            if is_day:
                marker, msize, medge, mwidth, malpha = cm._agent_marker, cm._agent_markersize_small, cm._agent_markeredgecolor, cm._agent_markeredgewidth, cm._agent_markeralpha
            else:
                marker, msize, medge, mwidth, malpha = cm._agent_marker_night, cm._agent_markersize_small_night, cm._agent_markeredgecolor_night, cm._agent_markeredgewidth_night, cm._agent_markeralpha_night
            
            for agent in self._schedule.agents:
                if isinstance(agent, BeaversVisualizerAgent):
                    x, y = agent._position
                    if 0 <= x < map_width and 0 <= y < map_height:
                        color = 'red' if agent._role == 'explorer' else 'black'
                        ax2.plot(x, y, marker, markersize=msize, markeredgecolor=medge, markerfacecolor=color, markeredgewidth=mwidth, alpha=malpha)
        
        # Plot home bases
        for agent in self._schedule.agents:
            if agent._home_base_position_store:
                for hx, hy in agent._home_base_position_store:
                    if 0 <= hx < map_width and 0 <= hy < map_height:
                        ax2.add_patch(plt.Rectangle((hx-2, hy-2), 3, 3, fill=True, edgecolor=cm._black, facecolor=cm._gray, linestyle='-', linewidth=2))
        
        # Configure axes
        titles = ["Vegetation Quality (Initial)", "Vegetation Quality (Current)", "Visits (Current)"]
        for ax, title in zip((ax1, ax2, ax3), titles):
            ax.set_aspect('equal')
            ax.grid(False)
            ax.set_xlabel('X [pixels]', fontsize=12)
            ax.set_ylabel('Y [pixels]', fontsize=12)
            ax.set_xlim(-0.5, map_width+0.5)
            ax.set_ylim(-0.5, map_height+0.5)
            ax.set_title(title)
        
        ax3.text(0.5, 1, f"DAY: {self._environment._current_day} HOUR: {self._environment._current_hour}h", 
                fontsize=14, color=cm._black, font='monospace')
        
        self._fig = fig
        if self._gui:
            plt.show()    
    
    
    def plot_simulation_recap(self) -> None:
        """Plot N×4 subplots: distance, load, error norm, and exploration eta for each agent."""
        # Get and organize agents
        agents = [a for a in self._schedule.agents if isinstance(a, BeaversVisualizerAgent)]
        explorers = [a for a in agents if hasattr(a, '_role') and a._role == 'explorer']
        expanders = [a for a in agents if hasattr(a, '_role') and a._role == 'expander']
        agent_list = (explorers + expanders) if (explorers or expanders) else agents
        
        if not agent_list:
            print("No agents found for plotting")
            return
        
        # Create figure
        n = len(agent_list)
        fig, axes = plt.subplots(n, 4, figsize=(25, 4 * n))
        if n == 1:
            axes = axes.reshape(1, -1)
        
        # Process each agent
        for i, agent in enumerate(agent_list):
            role = getattr(agent, '_role', 'unknown')
            positions, loads, errors, etas = agent._position_store, agent._load_store, agent._error_store, agent._exploration_eta_store
            
            if positions:
                init_pos = positions[0]
                time_steps = list(range(len(positions)))
                
                # Calculate distances and find river positions
                distances, river_times, river_dists = [], [], []
                for t, pos in enumerate(positions):
                    dist = self.np.sqrt((pos[0] - init_pos[0])**2 + (pos[1] - init_pos[1])**2)
                    distances.append(dist)
                    if hasattr(self, '_environment') and self._environment._map is not None:
                        x, y = int(pos[0]), int(pos[1])
                        if 0 <= x < self._environment._map.shape[0] and 0 <= y < self._environment._map.shape[1]:
                            if self._environment._map[x, y] < -2:
                                river_times.append(t)
                                river_dists.append(dist)
                
                # Distance plot
                axes[i, 0].plot(time_steps, distances, color='black', linewidth=2, alpha=0.8)
                if river_times:
                    axes[i, 0].scatter(river_times, river_dists, color='blue', marker='o', s=50, alpha=0.8, 
                                     edgecolors='darkblue', linewidth=1, label='In River')
                    axes[i, 0].legend()
                axes[i, 0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
                axes[i, 0].set_xlabel('Time Steps')
                axes[i, 0].set_ylabel('Distance from Start')
                axes[i, 0].set_title(f'Agent {agent.unique_id}: Distance from Initial Position')
                axes[i, 0].grid(True, alpha=0.3)
                axes[i, 0].text(0.95, 0.95, f'Role: {role.capitalize()}', transform=axes[i, 0].transAxes, fontsize=10,
                              va='top', ha='right', bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgray', alpha=0.8, edgecolor='black'))
                
                # Load plot
                axes[i, 1].plot(time_steps, loads, color='black', linewidth=2, alpha=0.8, label='Current Load')
                if hasattr(agent, '_maximum_load_store') and agent._maximum_load_store:
                    axes[i, 1].plot(time_steps[:len(agent._maximum_load_store)], agent._maximum_load_store, 
                                  color='red', linewidth=1.5, alpha=0.7, linestyle='--', label='Maximum Load Capacity')
                axes[i, 1].set_xlabel('Time Steps')
                axes[i, 1].set_ylabel('Load Amount')
                axes[i, 1].set_title(f'Agent {agent.unique_id}: Load Variation')
                axes[i, 1].grid(True, alpha=0.3)
                axes[i, 1].set_ylim(0, agent._maximum_load_init + 1)
                axes[i, 1].legend()
                
                # Error plot
                error_norms = [self.np.linalg.norm(e) if hasattr(e, '__len__') and len(e) > 0 else abs(e) if e is not None else 0.0 for e in errors]
                if error_norms:
                    axes[i, 2].plot(time_steps[:len(error_norms)], error_norms, color='black', linewidth=2, alpha=0.8)
                    axes[i, 2].set_xlabel('Time Steps')
                    axes[i, 2].set_ylabel('Error Norm')
                    axes[i, 2].set_title(f'Agent {agent.unique_id}: Control Error Norm')
                    axes[i, 2].grid(True, alpha=0.3)
                else:
                    axes[i, 2].text(0.5, 0.5, 'No error data available', ha='center', va='center', transform=axes[i, 2].transAxes)
                    axes[i, 2].set_title(f'Agent {agent.unique_id}: Control Error Norm')
                
                # Exploration eta plot
                if etas:
                    axes[i, 3].plot(time_steps[:len(etas)], etas, color='black', linewidth=2, alpha=0.8, label='Exploration Eta')
                    if hasattr(agent, '_harvest_threshold_store') and agent._harvest_threshold_store:
                        lower = [th[0] for th in agent._harvest_threshold_store if len(th) >= 2]
                        upper = [th[1] for th in agent._harvest_threshold_store if len(th) >= 2]
                        if lower and upper:
                            axes[i, 3].plot(time_steps[:len(lower)], lower, color='red', linewidth=1.5, alpha=0.7, linestyle='--', label='Harvest Threshold Min')
                            axes[i, 3].plot(time_steps[:len(upper)], upper, color='orange', linewidth=1.5, alpha=0.7, linestyle='--', label='Harvest Threshold Max')
                    axes[i, 3].set_xlabel('Time Steps')
                    axes[i, 3].set_ylabel('Values')
                    axes[i, 3].set_title(f'Agent {agent.unique_id}: Exploration Eta & Harvest Thresholds')
                    axes[i, 3].grid(True, alpha=0.3)
                    axes[i, 3].legend()
                    axes[i, 3].set_ylim(0, agent._vegetation_quality_range[1] + 0.5)
                else:
                    axes[i, 3].text(0.5, 0.5, 'No exploration eta data available', ha='center', va='center', transform=axes[i, 3].transAxes)
                    axes[i, 3].set_title(f'Agent {agent.unique_id}: Exploration Eta & Harvest Thresholds')
            else:
                # No data
                for j, title in enumerate([f'Agent {agent.unique_id}: Distance from Initial Position', 
                                          f'Agent {agent.unique_id}: Load Variation',
                                          f'Agent {agent.unique_id}: Control Error Norm', 
                                          f'Agent {agent.unique_id}: Exploration Eta & Harvest Thresholds']):
                    axes[i, j].text(0.5, 0.5, 'No data available', ha='center', va='center', transform=axes[i, j].transAxes)
                    axes[i, j].set_title(title)
                axes[i, 0].text(0.95, 0.95, f'Role: {role.capitalize()}', transform=axes[i, 0].transAxes, fontsize=10,
                              va='top', ha='right', bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgray', alpha=0.8, edgecolor='black'))
        
        plt.tight_layout()
        if self._gui:
            plt.show()

    
    def save_environment_map(self, file_path: str) -> None:
        """Save current environment map to .npy file with linear rescaling to [-1, 1]."""
        import os
        import numpy as np
        
        if self._environment is None:
            raise ValueError("Environment not initialized. Call generate_agents() first.")
        if self._environment._map is None:
            raise ValueError("Environment map not available.")
        
        # Create directory if needed
        directory = os.path.dirname(file_path)
        if directory and not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
        
        # Add .npy extension if missing
        if not file_path.endswith('.npy'):
            file_path += '.npy'
        
        # Linear rescaling to [-1, 1]
        map_data = self._environment._map.copy().astype(float)
        data_min = map_data.min()
        data_max = map_data.max()
        
        if data_max > data_min:
            map_data = (map_data - data_min) / (data_max - data_min) * 2 - 1
            normalization_info = f"[{data_min:.3f}, {data_max:.3f}] -> [-1.0, 1.0]"
        else:
            map_data = map_data * 0
            normalization_info = f"Constant {data_min:.3f} -> 0.0"
        
        # Apply inverse coordinate transformation (reverse of load_map_from_file)
        map_to_save = np.rot90(map_data, 1)
        map_to_save = np.flipud(map_to_save)
        
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
    """Beaver agent for visualization-based simulations."""
    
    def __init__(self, unique_id, model, **kwargs) -> None:
        BeaversRobotBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_robot(**kwargs)
        self._timedelta = model._timedelta
                    
    def step(self) -> None:
        """Execute one simulation step: get environment data, measure map, step beaver behavior."""
        # Get data from environment
        dt = self._timedelta
        time_of_day = self.model._environment._time_of_day
        measure_positions, measure_values = module_misc.measure(self.model._environment._map, self._position, self._measurement_mode, self._measure_step)
        map_quality = [measure_positions, measure_values]
        
        # Get map limits
        if self._measurement_mode is 'full_map':
            limits = module_misc.get_map_limits(self.model._environment._map)
        else:
            if self._local_map is not None:
                limits = module_misc.get_map_limits(self._local_map)
            else:
                limits = [[0, self._position[0] - 1], [0, self._position[1] - 1]]
        
        # Link environment data to agent
        self._vegetation_quality_range = self.model._environment._vegetation_quality_range
        self._range_x = self.model._environment._width
        self._range_y = self.model._environment._height
        self._home_base_position_store = self.model._environment._home_base_position_store
        
        # Link flow information
        misc = {
            'direction': self.model._environment._flow_direction,
            'strength': self.model._environment._flow_strength,
            'visits': self.model._environment._map_visits
        }
        
        # Step the agent
        self.step_beaver(dt, time_of_day, map_quality, limits, misc)
        
        # Update environment at current position
        self.model._environment._map_original[self._position[0], self._position[1]] = self._map_quality_measure_position

class EnvironmentVisualizerAgent(BeaversEnvironmentBackend, Agent):
    """Environment agent for visualization-based simulations."""
    
    def __init__(self, unique_id, model, **kwargs) -> None:
        BeaversEnvironmentBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_environment(**kwargs)
        self._timedelta = model._timedelta
                    
    def step(self) -> None:
        """Aggregate agent activities and update environmental state."""
        # Aggregate visit maps from all agents
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
                        map_visits_roles[agents._position[0], agents._position[1]] = -map_visits[agents._position[0], agents._position[1]]
                    else:
                        map_visits_roles[agents._position[0], agents._position[1]] = map_visits[agents._position[0], agents._position[1]]
                if agents._home_base_position_store is not None:
                    for pos in agents._home_base_position_store:
                        home_base_position_store.append(pos)
        
        map_visits = map_visits * self._visits_reset
        map_visits_roles = map_visits_roles * self._visits_reset
        misc = {'map_visits_roles': map_visits_roles}
        home_base_position_store = list(set(tuple(pos) for pos in self.np.array(home_base_position_store)))

        # Step the environment
        self.step_environment(self._timedelta, map, map_visits, home_base_position_store, self._grass_growth_interval, misc)