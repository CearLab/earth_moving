# This file contains the modules used in the backend of the RAL project

# import libraries
import matplotlib.colors as cc
from matplotlib.path import Path
import numpy as np

# Define a custom leaf-shaped marker
flag_marker = Path(
    np.array([
        (0, 0), (0, 1), (0.8, 1), (0.5, 0.7), (0.8, 0.4), (0, 0.4), (0, 0)
    ]) - (0.4, 0.5),
    [
        Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.MOVETO, Path.CLOSEPOLY
    ]
)

# define a battery-shaped marker
battery_marker = Path(
    np.array([
        (0, 0), (0, 0.9), (0.2, 0.9), (0.2, 1), (0.4, 1), (0.4, 0.9), (0.6, 0.9), (0.6, 0), (0, 0)
    ]) - (0.1, 0.5),
    [
        Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY
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
        self._agent_markersize_small = 10
        self._agent_markerfacecolor = '#936639' # brown
        self._agent_markeredgecolor = '#465362' # dark grey
        self._agent_markeredgewidth = 2
        self._agent_markeralpha = 0.8
        
        # agent markers - night
        self._agent_marker_night = 'ro'
        self._agent_markersize_night = 20
        self._agent_markersize_small_night = 10
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
        lightgreen_night = '#001845'
        mediumgreen_night = '#002855'
        green_night = '#023e7d'
        tangreen_night = '#0353a4'
        darkgreen_night = '#0466c8'
        alpha_green_night = 1.0
        
        # brown - daylight
        lightbrown = '#c9a66b'
        mediumbrown = '#a98b5e'
        brown = '#8a6f4e'
        tanbrown = '#7a5f4a'
        darkbrown = '#5f4a3c'
        alpha_brown = 0.6
        
        # brown - night
        lightbrown_night = '#979dac'
        mediumbrown_night = '#7d8597'
        brown_night = '#5c677d'
        tanbrown_night = '#33415c'
        darkbrown_night = '#001233'
        alpha_brown_night = 0.6
        
        # orange - quality        
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
        
        # battery markers
        self._battery_marker = battery_marker
        self._battery_markersize = 20
        self._battery_markeredgecolor = '#465362' # dark grey
        self._battery_markeredgewidth = 2
        self._battery_markeralpha = 1.0
        
        # general clolors
        self._white = '#FFFFFF'
        self._black = '#000000'                
        red = '#FF0000'                        
        
        # green colormap - daylight
        self._green_colors_list = [(0, self._background_color),
                                   (0.2, lightgreen),
                                   (0.4, mediumgreen),
                                   (0.6, green),
                                   (0.8, tangreen),
                                   (1, darkgreen)]
        self._green_colormap = cc.LinearSegmentedColormap.from_list("green_colormap",self._green_colors_list)
        self._green_colormap_alpha = alpha_green
        
        # green colormap - sleepnight
        self._green_colors_list_night = [(0, self._background_color_night),
                                         (0.2, lightgreen_night),
                                         (0.4, mediumgreen_night),
                                         (0.6, green_night),
                                         (0.8, tangreen_night),
                                         (1, darkgreen_night)]
        self._green_colormap_night = cc.LinearSegmentedColormap.from_list("green_colormap_night",self._green_colors_list_night)
        self._green_colormap_alpha_night = alpha_green_night
        
        # brown colormap - daylight
        self._brown_colors_list = [(0, self._background_color),
                                   (0.2, lightbrown),
                                   (0.4, mediumbrown),
                                   (0.6, brown),
                                   (0.8, tanbrown),
                                   (1, darkbrown)]
        self._brown_colormap = cc.LinearSegmentedColormap.from_list("brown_colormap",self._brown_colors_list)
        self._brown_colormap_alpha = alpha_brown
        
        # brown colormap - sleepnight
        self._brown_colors_list_night = [(0, self._background_color_night),
                                         (0.2, lightbrown_night),
                                         (0.4, mediumbrown_night),
                                         (0.6, brown_night),
                                         (0.8, tanbrown_night),
                                         (1, darkbrown_night)]
        self._brown_colormap_night = cc.LinearSegmentedColormap.from_list("brown_colormap_night",self._brown_colors_list_night)
        self._brown_colormap_alpha_night = alpha_brown_night
        
        # brown-to-green colormap - daylight
        self._browngreen_colors_list = [(0, darkbrown),
                                        (0.1, tanbrown),
                                        (0.2, brown),
                                        (0.3, mediumbrown),
                                        (0.4, lightbrown),
                                        (0.5, lightgreen),
                                        (0.6, mediumgreen),
                                        (0.7, green),
                                        (0.8, tangreen),
                                        (1, darkgreen)]
        self._browngreen_colormap = cc.LinearSegmentedColormap.from_list("browngreen_colormap",self._browngreen_colors_list)
        self._browngreen_colormap_alpha = alpha_brown
        
        # brown-to-green colormap - sleepnight
        self._browngreen_colors_list_night = [(0, darkbrown_night),
                                              (0.1, tanbrown_night),
                                              (0.2, brown_night),
                                              (0.3, mediumbrown_night),
                                              (0.4, lightbrown_night),
                                              (0.5, lightgreen_night),
                                              (0.6, mediumgreen_night),
                                              (0.7, green_night),
                                              (0.8, tangreen_night),
                                              (1, darkgreen_night)]
        self._browngreen_colormap_night = cc.LinearSegmentedColormap.from_list("browngreen_colormap_night",self._browngreen_colors_list_night)
        self._browngreen_colormap_alpha_night = alpha_brown_night
        
        # orange colormap
        self._orange_colors_list = [(0, self._white),
                                    (0.2, lightorange),
                                    (0.4, mediumorange),
                                    (0.6, orange),
                                    (0.8, tanorange),
                                    (1, darkorange)]
        self._orange_colormap = cc.LinearSegmentedColormap.from_list("orange_colormap",self._orange_colors_list)
        self._orange_colormap_alpha = alpha_orange
        
        # red-green colormap
        self._redgreen_colors_list = [  (0, red),
                                        (0.2, lightorange),
                                        (0.4, orange),
                                        (0.6, lightgreen),
                                        (0.8, mediumgreen),
                                        (1, green)]
        self._redgreen_colormap = cc.LinearSegmentedColormap.from_list("redgreen_colormap",self._redgreen_colors_list)
        self._redgreen_colormap_alpha = alpha_orange
                
                