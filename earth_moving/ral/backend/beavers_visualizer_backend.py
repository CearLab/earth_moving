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
class BeaversVisualizerBackend(BaseBackend,Model):
    
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
        
    # Plot the environment with trails and canals overlaid.
    def plot_environment_with_heatmap(self) -> None:
        
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
            map_colormap = self._color_maps._whiteblack_colormap
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
            map_colormap = self._color_maps._whiteblack_colormap
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
        # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 20))
        fig, (ax1) = plt.subplots(1, 1, figsize=(10, 10))
                
        # box around the environment        
        box_margin = 0.5
        box_left = plt.Rectangle((0, 0), self._width-1, self._height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        box_right = plt.Rectangle((0, 0), self._width-1, self._height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        ax1.add_patch(box_left)
        # ax2.add_patch(box_right)

        # Normalize map
        map_normalized = self._environment._map / v_normalizer
        map_visits_normalized = self._environment._map_visits / v_normalizer
        
        # Plot map with geographic coordinate ticks
        im = ax1.imshow(map_normalized.transpose(), origin='lower', 
                        cmap=map_colormap, alpha=alpha_map,
                        vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax1, shrink=0.8, aspect=20, pad=0.02)
        cbar.set_label('Vegetation Quality / Elevation', rotation=270, labelpad=20)
        # Set colorbar ticks to show actual values (not normalized)
        cbar_ticks = [vmin/v_normalizer, 0, vmax/v_normalizer]
        cbar_labels = [f'{vmin:.1f}', '0.0', f'{vmax:.1f}']
        cbar.set_ticks(cbar_ticks)
        cbar.set_ticklabels(cbar_labels)                
        
        # Add another imshow on the right        
        # ax2.imshow(map_visits_normalized.transpose(), origin='lower', 
        #        cmap=map_colormap, alpha=alpha_map,
        #        vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)        

        # Overlay agent positions
        if True:
            for agent in self._schedule.agents:
                if isinstance(agent, BeaversVisualizerAgent):
                    # Use pixel coordinates directly (simple approach)
                    ax1.plot(agent._position[0], agent._position[1], 
                        agent_marker, 
                        markersize=      agent_markersize, 
                        markeredgecolor= agent_markeredgecolor,
                        markerfacecolor= self._color_maps._black,
                        markeredgewidth= agent_markeredgewidth,
                        alpha=           agent_markeralpha)
                    
                    #! note here that we're linking the agent to the environment to get the map quality. This is why we need an engine
                    # try:
                    #     map_normalized = agent._map_quality_measure_position / (vmax - vmin)
                    # except:
                    #     map_normalized = self.np.nan
                        
                    # ax1.plot(agent._position[0] + 0.04 * self._width, agent._position[1], 
                    #     marker =         vegetation_marker.vertices, 
                    #     markersize=      vegetation_markersize, 
                    #     markeredgecolor= vegetation_markeredgecolor,
                    #     markerfacecolor= self._color_maps._orange_colormap(map_normalized),
                    #     markeredgewidth= vegetation_markeredgewidth,
                    #     alpha=           vegetation_markeralpha)
                    
                    # battery marker
                    # energy_normalized = agent._energy/100
                    # ax1.plot(agent._position[0] - 0.045 * self._width, agent._position[1], 
                    #     marker =         battery_marker.vertices, 
                    #     markersize=      battery_markersize, 
                    #     markeredgecolor= battery_markeredgecolor,
                    #     markerfacecolor= self._color_maps._redgreen_colormap(energy_normalized),
                    #     markeredgewidth= battery_markeredgewidth,
                    #     alpha=           battery_markeralpha)
                    
        for agent in self._schedule.agents:
            # add a box around home_position
            if agent._home_base_position_store is not None:
                for home_base_position in agent._home_base_position_store:
                    # Use pixel coordinates (simple approach)
                    box = plt.Rectangle((home_base_position[0] - 2, home_base_position[1] - 2), 3, 3, 
                                        fill=True, edgecolor=self._color_maps._black, facecolor=self._color_maps._gray, linestyle='-', linewidth=2)
                    ax1.add_patch(box)
        
        # set axes
        ax1.set_aspect('equal')
        ax1.grid(False)
        
        # Set custom tick labels with latitude/longitude if available
        if hasattr(self._environment, 'x_axis') and hasattr(self._environment, 'y_axis'):
            # Show geographic coordinate labels
            ax1.set_xlabel('X [m]', fontsize=12)
            ax1.set_ylabel('Y [m]', fontsize=12)

            # Create custom tick positions and labels
            # Sample 6 points across each axis for reasonable tick spacing
            n_ticks = 6
            x_tick_positions = self.np.linspace(0, len(self._environment.x_axis)-1, n_ticks, dtype=int)
            y_tick_positions = self.np.linspace(0, len(self._environment.y_axis)-1, n_ticks, dtype=int)

            # Get corresponding geographic coordinates
            x_tick_labels = [f'{self._environment.x_axis[pos]:.4f}' for pos in x_tick_positions]
            y_tick_labels = [f'{self._environment.y_axis[pos]:.4f}' for pos in y_tick_positions]
            
            # Set the ticks
            ax1.set_xticks(x_tick_positions)
            ax1.set_xticklabels(x_tick_labels)
            ax1.set_yticks(y_tick_positions)
            ax1.set_yticklabels(y_tick_labels)
        else:
            ax1.set_axis_off()
        
        # Set axis limits in pixel coordinates
        box_margin = 0.5
        ax1.set_xlim(0 - box_margin, self._width + box_margin)
        ax1.set_ylim(0 - box_margin, self._height + box_margin)
        ax1.set_title("Vegetation Heatmap")
                
        ax1.text(0.5, 1, f"DAY: {self._environment._current_day} HOUR: {self._environment._current_hour}h", 
                        fontsize=14, color=self._color_maps._white, font=fontname)        
        
        # ax2.set_aspect('equal')
        # ax2.grid(False)
        # ax2.set_axis_off()
        # ax2.set_xlim(0 - box_margin, self._width + box_margin)
        # ax2.set_ylim(0 - box_margin, self._height + box_margin)
        # ax2.set_title("Visits Heatmap")
                
        ## FIG - AGENTS     
        if False:   
            for agent in self._schedule.agents:
                if isinstance(agent, BeaversVisualizerAgent): 
                    
                    # Add a new row for axfo
                    ax = fig.add_axes([0.125, 0.2 - (0.2 *agent.unique_id), 0.353, 0.2])  # [left, bottom, width, height]                
                    
                    # box around the environment        
                    box_margin = 0.5
                    box = plt.Rectangle((0, 0), self._width-1, self._height-1, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
                    ax.add_patch(box)                
                    
                    # Normalize map
                    try:
                        map_normalized = agent._local_map / v_normalizer  
                        # Pad the map with zeros to match the environment dimensions
                        padded_map = self.np.ones((self._width, self._height)) * self.np.nan
                        padded_map[:map_normalized.shape[0], :map_normalized.shape[1]] = map_normalized
                        # Plot map
                        ax.imshow(padded_map.transpose(), origin='lower', 
                                cmap=map_colormap, alpha=alpha_map,
                                vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
                        map_print = agent._map_quality_measure_position
                    except:
                        ax.imshow(self.np.nan * self.np.ones((self._width, self._height)).transpose(), origin='lower', 
                                cmap=map_colormap, alpha=alpha_map,
                                vmin=vmin/v_normalizer, vmax=vmax/v_normalizer)
                        map_print = self.np.nan
                        
                    # Plot agent's position
                    ax.plot(agent._position[0], agent._position[1], 
                        agent_marker, 
                        markersize= 0.25 * agent_markersize, 
                        markeredgecolor=agent_markeredgecolor,
                        markerfacecolor=agent_markerfacecolor,
                        markeredgewidth=agent_markeredgewidth,
                        alpha=agent_markeralpha)
                    
                    # add a box around home_position
                    if agent._home_base_position is not None:
                        box = plt.Rectangle((agent._home_base_position[0] - 0.5, agent._home_base_position[1] - 0.5), 1, 1, 
                                            fill=False, edgecolor='lightgreen', facecolor='white', linestyle='-', linewidth=2)
                        ax.add_patch(box)
                    
                    # Draw an arrow from the agent's position to its destination
                    if agent._motion_destination is not None:
                        ax.arrow(agent._position[0], agent._position[1],
                                    agent._motion_destination[0] - agent._position[0],
                                    agent._motion_destination[1] - agent._position[1],
                                    head_width=0.5, head_length=0.7, fc=self._color_maps._black, ec=self._color_maps._black, alpha=1.0)
                                        
                    ax.set_aspect('equal')
                    ax.grid(False)
                    ax.set_axis_off()                
                    ax.set_xlim(0 - box_margin, self._width + box_margin)
                    ax.set_ylim(0 - box_margin, self._height + box_margin)
                    ax.set_title(f"Agent {agent.unique_id}: local map")
                    
                    # Add a second axis with different size/shape
                    panel = fig.add_axes([0.55, 0.21 - (0.2 * agent.unique_id), 0.35, 0.18])  # [left, bottom, width, height]
                    panel.set_axis_off()
                    box = plt.Rectangle((0.0, 0.0), 0.96, 0.96, 
                                        fill=True, edgecolor='black', 
                                        facecolor=self._color_maps._background_color_monitor, 
                                        linestyle='-', linewidth=1)
                    panel.add_patch(box)  
                    
                    panel.text(0.05, 0.9, f"AGENT {agent.unique_id}:", 
                        fontsize=14, color='black', verticalalignment='top', font=fontname)                        
                    panel.text(0.1, 0.8, 
                        f"TIME: {agent._current_time} ENERGY: {agent._energy:.1f}\n"
                        f"VEGETATION: {map_print:.1f} LOAD: {agent._load:.1f} \n"
                        f"POS: {self.np.array(agent._position)} DST: {self.np.array(agent._motion_destination)} CTRL: {agent._status_motion}\n"
                        f"TASK: {agent._current_task} STATUS: {agent._status_task} \n"
                        f"ROBOT STATUS: {agent._status_robot} ACTION: {agent._current_action}", 
                        fontsize=14, color='black', verticalalignment='top', font=fontname)                                                                                                                  
        self._fig = fig
        if self._gui:
            plt.show()    
        
class BeaversVisualizerAgent(BeaversRobotBackend, Agent):
    
    def __init__(self, unique_id, model, **kwargs) -> None:        
        BeaversRobotBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_robot(**kwargs)   
        
        # this section bridges the agent.step with the required fields from the environment/engine
        self._timedelta = model._timedelta
                    
    def step(self) -> None:        
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
        
        # step the agent
        self.step_beaver(dt, 
                         time_of_day,
                         map_quality, 
                         limits)
        
        #! here we update the environment with the agent's actions 
        #! note that we only update the current position because it's the only the beaver can actually change
        if self._map_quality_update == True:
            self.model._environment._map_original[self._position[0], self._position[1]] =self._map_quality_measure_position
        
class EnvironmentVisualizerAgent(BeaversEnvironmentBackend, Agent):
    
    def __init__(self, unique_id, model, **kwargs) -> None:                
        BeaversEnvironmentBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_environment(**kwargs)      
        self._timedelta = model._timedelta       
                    
    def step(self) -> None:     
        
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
               
        grass_growth_interval = [agents._harvest_threshold[0] + agents._vegetation_removal, self._vegetation_quality_init_range[1]]
        self.step_environment(self._timedelta, map_visits, home_base_position_store, grass_growth_interval)        