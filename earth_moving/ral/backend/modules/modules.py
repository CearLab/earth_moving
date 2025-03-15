# This file contains the modules used in the backend of the RAL project

# import libraries
import matplotlib.colors as cc

# class to define the color maps
class ColorMaps(): 
    
    def __init__(self) -> None:  
        
        # background color
        # self._background_color = '#FFFFFF' # white
        self._background_color = '#e9f5db' # nature green
        
        # agent markers
        self._agent_marker = 'ro'
        self._agent_markersize = 20
        self._agent_markerfacecolor = '#936639'
        self._agent_markeredgecolor = '#465362'
        self._agent_markeredgewidth = 2
        self._agent_markeralpha = 1.0
        
        # greens
        lightgreen = '#cfe1b9'
        mediumgreen = '#b5c99a'
        green = '#97a97c'
        tangreen = '#87986a'
        darkgreen = '#718355'
        alpha_green = 1.0
        
        # reds
        lightred = [1, 0.5, 0.5, 1]
        red = [1, 0, 0, 1]
        darkred = [0.5, 0, 0, 1]
        
        # blues
        lightblue = [0.5, 0.5, 1, 1]
        blue = [0, 0, 1, 1]
        darkblue = [0, 0, 0.5, 1]
        
        # green colormap
        self._green_colors_list = [(0, self._background_color),
                                   (0.2, lightgreen),
                                   (0.4, mediumgreen),
                                   (0.6, green),
                                   (0.8, tangreen),
                                   (1, darkgreen)]
        self._green_colormap = cc.LinearSegmentedColormap.from_list("green_colormap",self._green_colors_list)
        self._green_colormap_alpha = alpha_green
        
        # red colormap
        self._red_colors_list = [(0, self._background_color),
                                 (0.3, lightred),
                                 (0.6, red),
                                 (1, darkred)]
        self._red_colormap = cc.LinearSegmentedColormap.from_list("red_colormap",self._red_colors_list)
        
        # blue colormap
        self._blue_colors_list = [(0, self._background_color),
                                  (0.3, lightblue),
                                  (0.6, blue),
                                  (1, darkblue)]
        self._blue_colormap = cc.LinearSegmentedColormap.from_list("blue_colormap",self._blue_colors_list)