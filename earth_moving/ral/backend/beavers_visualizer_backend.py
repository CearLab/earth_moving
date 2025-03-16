import time as t
import matplotlib.pyplot as plt
import numpy as np

from mesa import Agent, Model
from mesa.time import RandomActivation
from mesa.space import MultiGrid

from ral.backend.base_backend import BaseBackend
from ral.robot.robot_beavers_backend import BeaversRobotBackend

from ral.backend.modules.modules import ColorMaps

import random
class BeaversVisualizerBackend(BaseBackend,Model):
    
    def __init__(self, **kwargs) -> None:    
        # set seed
        random.seed(self._seed)
                            
        # attributes from simulation
        self._kwargs = kwargs
        simulation = self._kwargs.get('simulation')
        self._timedelta = simulation.get('timedelta')
        self._gui = simulation.get('gui')
        self._N_agents = simulation.get('number_of_agents')
                
        # mesa init
        self._schedule = RandomActivation(self)        
        
        # other attributes
        self._current_time = 0
        
        # colormaps
        self._color_maps = ColorMaps()
        
    def read_environment(self,environment) -> None:
        # attributes from environment
        self._width = environment._width
        self._height = environment._height 
        
        # grid init
        self._grid = MultiGrid(self._width, self._height, torus=False) #! why are the dimension opposite than in the environment setup?
        
    def generate_agents(self, **kwargs) -> None:
        for i in range(self._N_agents):            
            agent = BeaversVisualizerAgent(i, self, **kwargs)
            self._grid.place_agent(agent, (agent._position[0], agent._position[1]))
            self._schedule.add(agent)
        
    def step(self) -> None:
        self._current_time += self._timedelta #!! the agents step synchronously (_current_time wait for all agents to step)
        self._schedule.step()                    
        t.sleep(self._timedelta)
        
    # Plot the environment with trails and canals overlaid.
    def plot_environment_with_heatmap(self, environment) -> None:
        
        # check if GUI is set
        if not self._gui:
            raise Warning("There is no GUI set. Do you really need to call this method?")
        
        # colors vegetation
        vegetation_colormap = self._color_maps._green_colormap
        alpha_vegetation = self._color_maps._green_colormap_alpha
        
        # colors trails        
        trail_color_map = self._color_maps._red_colormap
        alpha_trail = 0.0
        
        # colors canals
        canal_color_map = self._color_maps._blue_colormap
        alpha_canal = 0.0                                
        
        # define image
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # box around the environment        
        box_margin = 0.5
        box = plt.Rectangle((0, 0), self._width, self._height, fill=False, edgecolor='black', facecolor='white', linestyle='-', linewidth=2)
        ax.add_patch(box)                

        # Normalize vegetation map
        vegetation_map_normalized = environment._vegetation_map / environment._vegetation_map.max()                
        # Plot vegetation
        ax.imshow(vegetation_map_normalized, origin='upper', cmap=vegetation_colormap, alpha=alpha_vegetation)

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
                    self._color_maps._agent_marker, 
                    markersize=self._color_maps._agent_markersize, 
                    markeredgecolor=self._color_maps._agent_markeredgecolor,
                    markerfacecolor=self._color_maps._agent_markerfacecolor,
                    markeredgewidth=self._color_maps._agent_markeredgewidth,
                    alpha=self._color_maps._agent_markeralpha)                        
        
        # set axes
        ax.set_aspect('equal')
        ax.grid(False)
        ax.set_axis_off()                
        ax.set_xlim(0 - box_margin, self._width + box_margin)
        ax.set_ylim(0 - box_margin, self._height + box_margin)
        ax.set_title("Trail and Canal Heatmap")        
        
        if self._gui:
            plt.show()        
        
class BeaversVisualizerAgent(BeaversRobotBackend, Agent):
    
    def __init__(self, unique_id, model, **kwargs) -> None:
        # set seed
        random.seed(self._seed)
        
        BeaversRobotBackend.__init__(self, **kwargs)
        Agent.__init__(self, unique_id, model)
        self.initiate_robot(**kwargs)  
                    
    def step(self) -> None:
        print('t= {}: Agent {} is doing {}'.format(self.model._current_time, self.unique_id, self._current_action))
        self.step_beaver()