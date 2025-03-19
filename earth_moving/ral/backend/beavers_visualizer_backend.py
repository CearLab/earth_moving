import time as t
import matplotlib.pyplot as plt
import numpy as np

from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid

from ral.backend.base_backend import BaseBackend
from ral.robot.robot_beavers_backend import BeaversRobotBackend
from ral.environment.environment_beavers_backend import BeaversEnvironmentBackend

from ral.backend.modules.modules import ColorMaps
class BeaversVisualizerBackend(BaseBackend,Model):
    
    def __init__(self, **kwargs) -> None:            
                            
        # attributes from simulation
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._gui = simulation.get('gui')
        self._N_agents = simulation.get('number_of_agents')
        self._print = simulation.get('print')
                
        # mesa init
        self._schedule = RandomActivation(self)        
        
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
        self._environment.step()
        self._schedule.step()                    
        # t.sleep(self._timedelta)
        
    # Plot the environment with trails and canals overlaid.
    def plot_environment_with_heatmap(self) -> None:
        
        fontname = 'monospace'
        
        # check if GUI is set
        if not self._gui:
            raise Warning("There is no GUI set. Do you really need to call this method?")
        
        # colors 
        if self._environment._time_of_day == 'day':
            # agents
            agent_marker =          self._color_maps._agent_marker
            agent_markersize =      self._color_maps._agent_markersize
            agent_markerfacecolor = self._color_maps._agent_markerfacecolor
            agent_markeredgecolor = self._color_maps._agent_markeredgecolor
            agent_markeredgewidth = self._color_maps._agent_markeredgewidth
            agent_markeralpha =     self._color_maps._agent_markeralpha
            
            # vegetation
            vegetation_colormap = self._color_maps._green_colormap
            alpha_vegetation = self._color_maps._green_colormap_alpha                        
        else:
            # agents
            agent_marker =          self._color_maps._agent_marker_night
            agent_markersize =      self._color_maps._agent_markersize_night
            agent_markerfacecolor = self._color_maps._agent_markerfacecolor_night
            agent_markeredgecolor = self._color_maps._agent_markeredgecolor_night
            agent_markeredgewidth = self._color_maps._agent_markeredgewidth_night
            agent_markeralpha =     self._color_maps._agent_markeralpha_night
            
            # vegetation
            vegetation_colormap = self._color_maps._green_colormap_night
            alpha_vegetation = self._color_maps._green_colormap_alpha_night
            
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
        fig, ax1 = plt.subplots(figsize=(10, 10))
                
        # box around the environment        
        box_margin = 0.5
        box = plt.Rectangle((0, 0), self._width, self._height, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        ax1.add_patch(box)

        # Normalize vegetation map
        vegetation_map_normalized = self._environment._vegetation_map / (self._environment._vegetation_quality_range[1] - self._environment._vegetation_quality_range[0])
        # Plot vegetation
        ax1.imshow(vegetation_map_normalized.transpose(), origin='lower', cmap=vegetation_colormap, alpha=alpha_vegetation)        

        # Overlay agent positions
        for agent in self._schedule.agents:
            if isinstance(agent, BeaversVisualizerAgent):
                ax1.plot(agent._position[0], agent._position[1], 
                    agent_marker, 
                    markersize=      agent_markersize, 
                    markeredgecolor= agent_markeredgecolor,
                    markerfacecolor= agent_markerfacecolor,
                    markeredgewidth= agent_markeredgewidth,
                    alpha=           agent_markeralpha)
                
                #! note here that we're linking the agent to the environment to get the vegetation quality. This is why we need an engine
                vegetation_normalized = agent._vegetation_quality/(self._environment._vegetation_quality_range[1] - self._environment._vegetation_quality_range[0])
                ax1.plot(agent._position[0] + 4, agent._position[1], 
                    marker =         vegetation_marker.vertices, 
                    markersize=      vegetation_markersize, 
                    markeredgecolor= vegetation_markeredgecolor,
                    markerfacecolor= self._color_maps._orange_colormap(vegetation_normalized),
                    markeredgewidth= vegetation_markeredgewidth,
                    alpha=           vegetation_markeralpha)
                
                # battery marker
                energy_normalized = agent._energy/100
                ax1.plot(agent._position[0] - 4.2, agent._position[1], 
                    marker =         battery_marker.vertices, 
                    markersize=      battery_markersize, 
                    markeredgecolor= battery_markeredgecolor,
                    markerfacecolor= self._color_maps._redgreen_colormap(energy_normalized),
                    markeredgewidth= battery_markeredgewidth,
                    alpha=           battery_markeralpha)
        
        # set axes
        ax1.set_aspect('equal')
        ax1.grid(False)
        ax1.set_axis_off()                
        ax1.set_xlim(0 - box_margin, self._width + box_margin)
        ax1.set_ylim(0 - box_margin, self._height + box_margin)
        ax1.set_title("Trail and Canal Heatmap") 
        
        # Add a second axis with different size/shape
        panel = fig.add_axes([1, 0.3, 0.6, 0.4])  # [left, bottom, width, height]
        panel.set_axis_off()
        box = plt.Rectangle((0.0, 0.0), 0.96, 0.96, 
                            fill=True, edgecolor='black', 
                            facecolor=self._color_maps._background_color_monitor, 
                            linestyle='-', linewidth=1)
        panel.add_patch(box)     
        # Add text information to the panel
        panel.text(0.05, 0.9, f"ENGINE:", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.1, 0.85, f"t: {self._current_time}", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.1, 0.80, f"dt: {self._timedelta}", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.1, 0.75, f"N agents: {self._N_agents}", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.05, 0.65, f"ENVIRONMENT:", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.1, 0.60, f"day: {self._environment._current_day:.0f} hour: {self._environment._current_hour:.0f} light: {self._environment._time_of_day}", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.1, 0.55, f"dimensions: {self._environment._width} x {self._environment._height}", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        panel.text(0.1, 0.50, f"vegetation clusters: {self._environment._number_vegetation_clusters}", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)                
                
        ## FIG - AGENTS        

        for agent in self._schedule.agents:
            if isinstance(agent, BeaversVisualizerAgent): 
                
                # Add a new row for axfo
                ax = fig.add_axes([0.12, -0.15 - (0.45 *agent.unique_id), 0.78, 0.4])  # [left, bottom, width, height]                
                
                # box around the environment        
                box_margin = 0.5
                box = plt.Rectangle((0, 0), self._width, self._height, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
                ax.add_patch(box)                
                
                # Normalize vegetation map
                try:
                    vegetation_map_normalized = agent._local_map_vegetation / (self._environment._vegetation_quality_range[1] - self._environment._vegetation_quality_range[0])
                    # Pad the vegetation map with zeros to match the environment dimensions
                    padded_vegetation_map = np.ones((self._width, self._height)) * np.nan
                    padded_vegetation_map[:vegetation_map_normalized.shape[0], :vegetation_map_normalized.shape[1]] = vegetation_map_normalized
                    # Plot vegetation
                    ax.imshow(padded_vegetation_map.transpose(), origin='lower', cmap=vegetation_colormap, alpha=alpha_vegetation)
                except:
                    ax.imshow(np.nan * np.ones((self._width, self._height)).transpose(), origin='lower', cmap=vegetation_colormap, alpha=alpha_vegetation)
                    
                # Plot agent's position
                ax.plot(agent._position[0], agent._position[1], 
                    agent_marker, 
                    markersize= 1 * agent_markersize, 
                    markeredgecolor=agent_markeredgecolor,
                    markerfacecolor=agent_markerfacecolor,
                    markeredgewidth=agent_markeredgewidth,
                    alpha=agent_markeralpha)                
                                    
                ax.set_aspect('equal')
                ax.grid(False)
                ax.set_axis_off()                
                ax.set_xlim(0 - box_margin, self._width + box_margin)
                ax.set_ylim(0 - box_margin, self._height + box_margin)
                ax.set_title(f"Agent {agent.unique_id}: local vegetation map")
                
                # Add a second axis with different size/shape
                panel = fig.add_axes([1, -0.14 - (0.45 * agent.unique_id), 0.6, 0.4])  # [left, bottom, width, height]
                panel.set_axis_off()
                box = plt.Rectangle((0.0, 0.0), 0.96, 0.96, 
                                    fill=True, edgecolor='black', 
                                    facecolor=self._color_maps._background_color_monitor, 
                                    linestyle='-', linewidth=1)
                panel.add_patch(box)  
                
                panel.text(0.05, 0.9, f"AGENT {agent.unique_id}:", 
                    fontsize=14, color='black', verticalalignment='top', font=fontname)                        
                panel.text(0.1, 0.8, 
                    f"Time: {agent._current_time} Energy: {agent._energy:.1f}\n"
                    f"Vegetation: {agent._vegetation_quality:.1f} Load: {agent._load:.1f} \n"
                    f"POS: {np.array(agent._position)} DST: {np.array(agent._motion_destination)} CTRL: {agent._status_motion}\n"
                    f"TASK: {agent._current_task} ST: {agent._status_task} \n"
                    f"ACTION: {agent._current_action} STATUS: {agent._status_robot}", 
                    fontsize=14, color='black', verticalalignment='top', font=fontname)                                                                                   
        
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
        vegetation_quality = self.model._environment._vegetation_map[self._position[0], self._position[1]]
        limits = np.array([[0, self.model._environment._width-1], [0, self.model._environment._height-1]])
        
        # step the agent
        self.step_beaver(dt, 
                         time_of_day,
                         vegetation_quality, 
                         limits)        
        
class EnvironmentVisualizerAgent(BeaversEnvironmentBackend, Agent):
    
    def __init__(self, unique_id, model, **kwargs) -> None:                
        BeaversEnvironmentBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_environment(**kwargs)      
        self._timedelta = model._timedelta       
                    
    def step(self) -> None:            
        self.step_environment(self._timedelta)        