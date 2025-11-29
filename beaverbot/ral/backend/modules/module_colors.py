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
        
        # greens - daylight
        lightgreen = '#cfe1b9'
        mediumgreen = '#b5c99a'
        green = '#97a97c'
        tangreen = '#87986a'
        darkgreen = '#718355'
        alpha_green = 1.0
        
        # greens - night
        lightgreen_night = '#066839'
        mediumgreen_night = '#0a5c36'
        green_night = '#0f5132'
        tangreen_night = '#14452f'
        darkgreen_night = '#18392b'                
        alpha_green_night = 1.0
        
        # brown - daylight
        lightbrown = '#c9a66b'
        mediumbrown = '#a98b5e'
        brown = '#8a6f4e'
        tanbrown = '#7a5f4a'
        darkbrown = '#5f4a3c'
        alpha_brown = 0.6
        
        # brown - night
        lightbrown_night = '#4a2419'
        mediumbrown_night = '#411d13'
        brown_night = '#38160d'
        tanbrown_night = '#2f0e07'
        darkbrown_night = '#260701'
        alpha_brown_night = 0.6
        
        # blues - daylight
        lightblue = '#a9d6e5'
        mediumblue = '#89c2d9'
        blue = '#61a5c2'
        tanblue = '#468faf'
        darkblue = '#2c7da0'
        alpha_blue = 1.0
        
        # blues - night
        lightblue_night = '#00607a'
        mediumblue_night = '#005066'
        blue_night = '#004052'
        tanblue_night = '#00303d'
        darkblue_night = '#002029'
        alpha_blue_night = 1.0
        
        # orange - quality        
        lightorange = '#f2dc96'
        mediumorange = '#efcd5d'
        orange = '#d3b44e'
        tanorange = '#d8eaab'
        darkorange = '#95d387'
        alpha_orange = 1.0     
        
        # general clolors
        self._white = '#FFFFFF'
        self._black = '#000000' 
        self._light_gray = '#F8F9FA'  # light grey       
        self._gray = '#6c757d'  # grey        
        red = '#FF0000'    
        
        # background color
        # self._background_color = '#FFFFFF' # white
        self._background_color = '#e9f5db' # nature green
        self._background_color_night = '#979dac' # grey
        self._background_color_monitor = '#f8f9fa' # light grey
        
        # agent markers
        self._agent_marker = 'ro'
        self._agent_markersize = 20
        self._agent_markersize_small = 3
        self._agent_markerfacecolor = '#936639' # brown
        self._agent_markeredgecolor = self._light_gray # light grey
        self._agent_markeredgewidth = 1
        self._agent_markeralpha = 0.8
        
        # agent markers - night
        self._agent_marker_night = 'ro'
        self._agent_markersize_night = 20
        self._agent_markersize_small_night = 10
        self._agent_markerfacecolor_night = '#f8f9fa' # light grey
        self._agent_markeredgecolor_night = self._black # black
        self._agent_markeredgewidth_night = 1
        self._agent_markeralpha_night = 1.0                                               
        
        # vegetation markers
        self._vegetation_marker = flag_marker
        self._vegetation_markersize = 20
        self._vegetation_markeredgecolor = self._light_gray # light grey
        self._vegetation_markeredgewidth = 1
        self._vegetation_markeralpha = 1.0
        
        # battery markers
        self._battery_marker = battery_marker
        self._battery_markersize = 20
        self._battery_markeredgecolor = self._light_gray # light grey
        self._battery_markeredgewidth = 1
        self._battery_markeralpha = 1.0   
        
        self._visits_color_list = [ (0.00, blue),             
                                    (0.40, blue),                       
                                    (0.499, self._light_gray),
                                    (0.501, self._light_gray),
                                    (0.60, red),
                                    (1.00, red)]
        self._visits_colormap = cc.LinearSegmentedColormap.from_list("visits_colormap", self._visits_color_list)             
        
        # green colormap - daylight (handles -1 to 1 range: negatives=blue, positives=green)
        # Note: matplotlib expects 0-1, so -1 maps to 0.0, 0 maps to 0.5, +1 maps to 1.0
        self._green_colors_list = [(0.0, darkblue),      # -1 (invalid/water)
                                   (0.1, lightgreen),     # -0.5
                                   (0.5, mediumgreen),     # 0 (lowest valid elevation)
                                   (0.6, green),     # 0.2
                                   (0.7, tangreen),    # 0.4
                                   (0.8, darkgreen),          # 0.6
                                   (0.9, mediumbrown),       # 0.8
                                   (1.0, darkbrown)]      # 1.0 (highest elevation)
        self._green_colormap = cc.LinearSegmentedColormap.from_list("green_colormap",self._green_colors_list)
        self._green_colormap_alpha = alpha_green
        
        # blue to brown to green colormap - daylight
        self._bluebrowngreen_colors_list = [(0.00, darkblue),
                                            (0.10, tanblue),
                                            (0.15, blue),
                                            (0.20, blue),
                                            (0.25, mediumblue),
                                            (0.30, mediumblue),
                                            (0.40, lightblue),
                                            (0.499, lightblue),
                                            (0.501, lightgreen),
                                            (0.60, lightgreen),
                                            (0.70, mediumgreen),
                                            (0.80, tangreen),
                                            (0.85, tangreen),
                                            (0.95, darkgreen),
                                            (1.00, mediumbrown)]        
        self._bluebrowngreen_colormap = cc.LinearSegmentedColormap.from_list("bluebrowngreen_colormap",self._bluebrowngreen_colors_list)
        self._bluebrowngreen_colormap_alpha = alpha_brown
        
        # white to black colormap
        self._whiteblack_colors_list = [(0, self._white),
                                        (0.5, self._light_gray),                                        
                                        (1, self._black)]
        self._whiteblack_colormap = cc.LinearSegmentedColormap.from_list("whiteblack_colormap",self._whiteblack_colors_list)
        self._whiteblack_colormap_alpha = 1.0
        
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
                
                