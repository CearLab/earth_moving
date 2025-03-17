import time as t
import matplotlib.pyplot as plt

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
        t.sleep(self._timedelta)
        
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
            
            # colors trails        
            trail_color_map = self._color_maps._red_colormap
            alpha_trail = 0.0
            
            # colors canals
            canal_color_map = self._color_maps._blue_colormap
            alpha_canal = 0.0
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
        #! vegetation_markerfacecolor defined in the ax.plot (it's a colormap)
        vegetation_markeredgecolor = self._color_maps._vegetation_markeredgecolor
        vegetation_markeredgewidth = self._color_maps._vegetation_markeredgewidth
        vegetation_markeralpha = self._color_maps._vegetation_markeralpha
                                        
        # define image
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # box around the environment        
        box_margin = 0.5
        box = plt.Rectangle((0, 0), self._width, self._height, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        ax.add_patch(box)

        # Normalize vegetation map
        vegetation_map_normalized = self._environment._vegetation_map / self._environment._vegetation_map.max()                
        # Plot vegetation
        ax.imshow(vegetation_map_normalized.transpose(), origin='lower', cmap=vegetation_colormap, alpha=alpha_vegetation)

        # Overlay trail heatmap
        # heatmap = np.log1p(environment._trail_usage_map) # Log scaling
        # ax.imshow(heatmap, cmap=trail_color_map, alpha=alpha_trail, origin='upper')

        # Overlay canal locations in blue
        # canal_overlay = np.zeros_like(environment._canal_usage_map)
        # canal_overlay[environment._canal_usage_map > 0] = 1  # Mark canals
        # ax.imshow(canal_overlay, cmap=canal_color_map, alpha=alpha_canal, origin='upper')

        # Overlay agent positions
        for agent in self._schedule.agents:
            if isinstance(agent, BeaversVisualizerAgent):
                ax.plot(agent._position[0], agent._position[1], 
                    agent_marker, 
                    markersize=      agent_markersize, 
                    markeredgecolor= agent_markeredgecolor,
                    markerfacecolor= agent_markerfacecolor,
                    markeredgewidth= agent_markeredgewidth,
                    alpha=           agent_markeralpha)
                
                vegetation_normalized = agent._vegetation_quality/(agent._vegetation_quality_range[1] - agent._vegetation_quality_range[0])
                ax.plot(agent._position[0] + 4.5, agent._position[1], 
                    marker =         vegetation_marker.vertices, 
                    markersize=      vegetation_markersize, 
                    markeredgecolor= vegetation_markeredgecolor,
                    markerfacecolor= self._color_maps._orange_colormap(vegetation_normalized),
                    markeredgewidth= vegetation_markeredgewidth,
                    alpha=           vegetation_markeralpha)
        
        # set axes
        ax.set_aspect('equal')
        ax.grid(False)
        ax.set_axis_off()                
        ax.set_xlim(0 - box_margin, self._width + box_margin)
        ax.set_ylim(0 - box_margin, self._height + box_margin)
        ax.set_title("Trail and Canal Heatmap") 
        
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
        
        panel.text(0.05, 0.40, f"AGENTS:", 
               fontsize=12, color='black', verticalalignment='top', font=fontname)
        for agent in self._schedule.agents:
            if isinstance(agent, BeaversVisualizerAgent):
                panel.text(0.1, 0.35 - 0.05 * agent.unique_id, 
                    f"A: {agent.unique_id} T: {agent._current_time} POS: ({agent._position[0]:.1f}, {agent._position[1]:.1f}) ACT: {agent._current_action} VEG: {agent._vegetation_quality:.1f}", 
                    fontsize=12, color='black', verticalalignment='top', font=fontname)
        
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
        # get data from engine/environment
        dt = self._timedelta
        time_of_day = self.model._environment._time_of_day
        vegetation_quality = self.model._environment._vegetation_map[self._position[0], self._position[1]]
        
        # step the agent
        self.step_beaver(dt, 
                         time_of_day,
                         vegetation_quality)        
        
class EnvironmentVisualizerAgent(BeaversEnvironmentBackend, Agent):
    
    def __init__(self, unique_id, model, **kwargs) -> None:                
        BeaversEnvironmentBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_environment(**kwargs)      
        self._timedelta = model._timedelta       
                    
    def step(self) -> None:            
        self.step_environment(self._timedelta)        