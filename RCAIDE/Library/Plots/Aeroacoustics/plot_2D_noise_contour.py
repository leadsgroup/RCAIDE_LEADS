# RCAIDE/Library/Plots/Noise/plot_2D_noise_contour.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units  

# python imports 
import numpy as np   
import matplotlib.pyplot as plt  
import matplotlib.colors
import matplotlib.colors as colors  

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_2D_noise_contour(noise_data,
                       noise_level = None,
                       min_noise_level = 35,  
                       max_noise_level = 90, 
                       noise_scale_label = None,
                       save_figure = False,
                       show_figure = True,
                       save_filename = "2D_Noise_Contour",
                       show_elevation = False,
                       use_lat_long_coordinates = False,  
                       colormap = 'jet',
                       file_type = ".png",
                       width = 10, 
                       height = 7):
    """
    Creates a 2D contour plot of noise levels with optional terrain elevation.

    Parameters
    ----------
    noise_data : NoiseData
        RCAIDE noise data structure containing:
            - microphone_locations[:,:,0:3]
                3D array of microphone positions
            - microphone_coordinates[:,:,0:2]
                Latitude and longitude of measurement points
            - topography_file
                Optional path to terrain elevation data
            
    noise_level : ndarray, optional
        2D array of noise levels at measurement points
        
    min_noise_level : float, optional
        Minimum noise level for contour scale (default: 35 dB)
        
    max_noise_level : float, optional
        Maximum noise level for contour scale (default: 90 dB)
        
    noise_scale_label : str, optional
        Label for noise metric (e.g., "dBA", "EPNL", etc.)
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_figure : bool, optional
        Flag to display the figure (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "2D_Noise_Contour")
        
    show_elevation : bool, optional
        Flag to display terrain elevation contours (default: False)
        
    use_lat_long_coordinates : bool, optional
        Flag to use geographic coordinates (default: True)
        
    colormap : str, optional
        Colormap specification for noise contours (default: 'jet')
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 10)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : plotly.graph_objects.Figure
        Handle to the generated figure

    Notes
    -----
    Creates visualization showing:
        - Noise level contours
        - Optional terrain elevation
        - Geographic or Cartesian coordinates
        - Customizable color scales
    
    **Definitions**
    
    'Noise Level'
        Sound pressure level or derived metric
    'Terrain Elevation'
        Height above sea level
    'Geographic Coordinates'
        Latitude and longitude positions
    
    See Also
    --------
    RCAIDE.Library.Plots.Noise.plot_3D_noise_contour : 3D visualization of noise field
    RCAIDE.Library.Plots.Noise.post_process_noise_data : Data preparation for plotting
    """      
    
    elevation       = noise_data.microphone_locations[:,:,2]/Units.ft      
    colors_undersea = plt.cm.terrain(np.linspace(0, 0.17, 56))
    colors_land     = plt.cm.terrain(np.linspace(0.25, 1, 200))  
    colors          = np.vstack((colors_undersea, colors_land))
    cut_terrain_map = matplotlib.colors.LinearSegmentedColormap.from_list('cut_terrain', colors) 
    norm = FixPointNormalize(sealevel=0,vmax=np.max(elevation),vmin=np.min(elevation))  
     
    fig = plt.figure(save_filename)
    fig.set_size_inches(width,height)
    
    axis = fig.add_subplot(1,1,1) 
    
    noise_levels   = np.linspace(min_noise_level,max_noise_level,10)  
    noise_cmap     = plt.get_cmap('turbo')
    noise_new_cmap = truncate_colormap(noise_cmap,0.0, 1.0) 
     
    if use_lat_long_coordinates:
        LAT  = noise_data.microphone_coordinates[:,:,0]
        LONG = noise_data.microphone_coordinates[:,:,1]
        axis.set_xlabel('Longitude [°]')
        axis.set_ylabel('Latitude [°]') 
    else:
        LAT  = noise_data.microphone_locations[:,:,0]
        LONG = noise_data.microphone_locations[:,:,1]
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')  
    
    if show_elevation:
        CS_1  = axis.contourf(LONG,LAT,elevation,cmap =cut_terrain_map,norm=norm,levels = 20, alpha=0.5)  
        cbar = fig.colorbar(CS_1, ax=axis)     
        cbar.ax.set_ylabel('Elevation above sea level [ft]', rotation =  90)  

    # plot aircraft noise levels   
    CS_2    = axis.contourf(LONG,LAT,noise_level ,noise_levels,cmap = noise_new_cmap)     
    cbar    = fig.colorbar(CS_2, ax=axis)        
    cbar.ax.set_ylabel(noise_scale_label, rotation =  90) 
        
    fig.tight_layout()  
    if save_figure: 
        figure_title  = save_filename
        plt.savefig(figure_title + file_type )     
         
    return fig       

# ------------------------------------------------------------------ 
# Truncate colormaps
# ------------------------------------------------------------------  
def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap


class FixPointNormalize(matplotlib.colors.Normalize):
    """ 
    Inspired by https://stackoverflow.com/questions/20144529/shifted-colorbar-matplotlib
    Subclassing Normalize to obtain a colormap with a fixpoint 
    somewhere in the middle of the colormap.
    This may be useful for a `terrain` map, to set the "sea level" 
    to a color in the blue/turquise range. 
    """
    def __init__(self, vmin=None, vmax=None, sealevel=0, col_val = 0.21875, clip=False):
        # sealevel is the fix point of the colormap (in data units)
        self.sealevel = sealevel
        # col_val is the color value in the range [0,1] that should represent the sealevel.
        self.col_val = col_val
        matplotlib.colors.Normalize.__init__(self, vmin, vmax, clip)

    def __call__(self, value, clip=None):
        x, y = [self.vmin, self.sealevel, self.vmax], [0, self.col_val, 1]
        return np.ma.masked_array(np.interp(value, x, y))   

def colorax(vmin, vmax):
    return dict(cmin=vmin,cmax=vmax)