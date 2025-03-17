# This file contains the modules used in the backend of the RAL project

# import libraries
import matplotlib.colors as cc
from matplotlib.path import Path
import numpy as np

# Define a custom leaf-shaped marker
flag_marker = Path(
    np.array([
        (0, 0), (0, 1), (0.8, 1), (0.5, 0.7), (0.8, 0.4), (0, 0.4), (0, 0)
    ]) - 0.5,
    [
        Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.MOVETO, Path.CLOSEPOLY
    ]
)

# class to define the color maps
class ColorMaps(): 
    
    def __init__(self) -> None:  
        
        # background color
        # self._background_color = '#FFFFFF' # white
        self._background_color = '#e9f5db' # nature green
        self._background_color_night = '#979dac' # grey
        self._background_color_monitor = '#f8f9fa' # light grey
        
        # agent markers
        self._agent_marker = 'ro'
        self._agent_markersize = 20
        self._agent_markerfacecolor = '#936639' # brown
        self._agent_markeredgecolor = '#465362' # dark grey
        self._agent_markeredgewidth = 2
        self._agent_markeralpha = 0.8
        
        # agent markers - night
        self._agent_marker_night = 'ro'
        self._agent_markersize_night = 20
        self._agent_markerfacecolor_night = '#f8f9fa' # light grey
        self._agent_markeredgecolor_night = '#465362' # dark grey
        self._agent_markeredgewidth_night = 2
        self._agent_markeralpha_night = 1.0
        
        # greens - daylight
        lightgreen = '#cfe1b9'
        mediumgreen = '#b5c99a'
        green = '#97a97c'
        tangreen = '#87986a'
        darkgreen = '#718355'
        alpha_green = 1.0
        
        # greens - night
        lightgreen_night = '#7d8597'
        mediumgreen_night = '#5c677d'
        green_night = '#33415c'
        tangreen_night = '#001233'
        darkgreen_night = '#001845'
        alpha_green_night = 1.0
        
        # orange - quality
        white = '#FFFFFF'
        lightorange = '#f2dc96'
        mediumorange = '#efcd5d'
        orange = '#d3b44e'
        tanorange = '#d8eaab'
        darkorange = '#95d387'
        alpha_orange = 1.0
        
        # vegetation markers
        self._vegetation_marker = flag_marker
        self._vegetation_markersize = 20
        self._vegetation_markeredgecolor = '#465362' # dark grey
        self._vegetation_markeredgewidth = 2
        self._vegetation_markeralpha = 1.0
        
        # reds
        lightred = [1, 0.5, 0.5, 1]
        red = [1, 0, 0, 1]
        darkred = [0.5, 0, 0, 1]
        
        # blues
        lightblue = [0.5, 0.5, 1, 1]
        blue = [0, 0, 1, 1]
        darkblue = [0, 0, 0.5, 1]
        
        # green colormap - daylight
        self._green_colors_list = [(0, self._background_color),
                                   (0.2, lightgreen),
                                   (0.4, mediumgreen),
                                   (0.6, green),
                                   (0.8, tangreen),
                                   (1, darkgreen)]
        self._green_colormap = cc.LinearSegmentedColormap.from_list("green_colormap",self._green_colors_list)
        self._green_colormap_alpha = alpha_green
        
        # green colormap - night
        self._green_colors_list_night = [(0, self._background_color_night),
                                         (0.2, lightgreen_night),
                                         (0.4, mediumgreen_night),
                                         (0.6, green_night),
                                         (0.8, tangreen_night),
                                         (1, darkgreen_night)]
        self._green_colormap_night = cc.LinearSegmentedColormap.from_list("green_colormap_night",self._green_colors_list_night)
        self._green_colormap_alpha_night = alpha_green_night
        
        # orange colormap
        self._orange_colors_list = [(0, white),
                                    (0.2, lightorange),
                                    (0.4, mediumorange),
                                    (0.6, orange),
                                    (0.8, tanorange),
                                    (1, darkorange)]
        self._orange_colormap = cc.LinearSegmentedColormap.from_list("orange_colormap",self._orange_colors_list)
        self._orange_colormap_alpha = alpha_orange
        
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