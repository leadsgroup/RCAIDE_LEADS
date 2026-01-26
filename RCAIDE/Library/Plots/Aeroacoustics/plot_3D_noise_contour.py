# RCAIDE/Library/Plots/Noise/plot_3D_noise_contour.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots  import *

# python imports 
import numpy as np  
import plotly.graph_objects as go

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------    
def plot_3D_noise_contour(noise_data,
                       noise_level = None,
                       min_noise_level = 35,  
                       max_noise_level = 90, 
                       noise_scale_label = None,
                       save_figure = False,
                       show_figure = True,
                       save_filename = "Noise_Contour",
                       use_lat_long_coordinates = True, 
                       show_trajectory = False,
                       show_microphones = False,
                       colormap = 'jet',
                       file_type = ".png",
                       background_color = 'rgb(17,54,71)',
                       grid_color = 'gray',
                       width = 1400, 
                       height = 800):
    """
    Creates an interactive 3D visualization of noise contours with optional aircraft trajectory.

    Parameters
    ----------
    noise_data : NoiseData
        RCAIDE noise data structure containing:
            - microphone_locations[:,:,0:3]
                3D array of microphone positions in (nmi, nmi, ft)
            - aircraft_position[:,0:3]
                Aircraft trajectory points in (nmi, nmi, ft)
            
    noise_level : ndarray
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
        Name of file for saved figure (default: "Noise_Contour")
        
    use_lat_long_coordinates : bool, optional
        Flag to use geographic coordinates (default: True)
        
    show_trajectory : bool, optional
        Flag to display aircraft trajectory (default: False)
        
    show_microphones : bool, optional
        Flag to display microphone locations (default: False)
        
    colormap : str, optional
        Colormap specification for noise contours (default: 'jet')
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    background_color : str, optional
        Color specification for plot background (default: 'rgb(17,54,71)')
        
    grid_color : str, optional
        Color specification for grid lines (default: 'gray')
        
    width : int, optional
        Figure width in pixels (default: 1400)
        
    height : int, optional
        Figure height in pixels (default: 800)

    Returns
    -------
    fig_3d : plotly.graph_objects.Figure
        Handle to the generated interactive 3D figure

    Notes
    -----
    Creates visualization showing:
        * 3D noise contour surface
        * Optional aircraft trajectory
        * Optional microphone locations
        * Interactive viewing controls
        * Customizable appearance
    
    **Major Assumptions**
        * Noise levels are in decibels
        * Coordinates are in nautical miles and feet
        * Measurement grid is regularly spaced
        * Z-axis represents elevation
    
    **Definitions**
    
    'Noise Contour'
        Surface of constant noise level
    'Aircraft Trajectory'
        Time history of aircraft position
    'Microphone Location'
        Measurement point coordinates
    
    See Also
    --------
    RCAIDE.Library.Plots.Noise.plot_2D_noise_contour : 2D visualization of noise field
    RCAIDE.Library.Plots.Noise.contour_surface_slice : Surface generation utility
    """   
    Aircraft_pos    = noise_data.aircraft_position      
    X               = noise_data.microphone_locations[:,:,0]/Units.nmi  
    Y               = noise_data.microphone_locations[:,:,1]/Units.nmi  
    Z               = noise_data.microphone_locations[:,:,2]/Units.feet  
    plot_data       = []   
  
    # ---------------------------------------------------------------------------
    # TRHEE DIMENSIONAL NOISE CONTOUR
    # --------------------------------------------------------------------------- 
    # TERRAIN CONTOUR 
    ground_contour   = contour_surface_slice(Y,X,Z,noise_level,color_scale=colormap)
    plot_data.append(ground_contour)

    # GROUND MICROPHONES
    if show_microphones:
        microphones = go.Scatter3d(x        = Y.flatten(),
                                   y        = X.flatten(),
                                   z        = Z.flatten(),
                                   mode     = 'markers',
                                   marker   = dict(size=6,color='white',opacity=0.8),
                                   line     = dict(color='white',width=2))
        plot_data.append(microphones)

    # AIRCRAFT TRAJECTORY
    if show_trajectory:
        aircraft_trajectory = go.Scatter3d(x   = Aircraft_pos[:,1]/Units.nmi,
                                           y   = Aircraft_pos[:,0]/Units.nmi,
                                           z   = Aircraft_pos[:,2]/Units.feet,
                                           mode= 'markers',
                                           marker=dict(size=6,
                                                       color='black',
                                                       opacity=0.8),
                                    line=dict(color='black',width=2))
        plot_data.append(aircraft_trajectory)

    # Define Colorbar Bounds
    min_alt     = np.minimum(np.min(Z),0) 
    max_alt     = np.maximum(np.max(Z), np.max(Aircraft_pos[:,2]/Units.feet)) 
  
    fig_3d = go.Figure(data=plot_data) 

    if show_microphones or show_trajectory:
        pass
    else: 
        fig_3d.update_traces(colorbar_orientation     = 'v',
                             colorbar_thickness       = 50,
                             colorbar_nticks          = 10,
                             colorbar_title_text      = noise_scale_label,
                             colorbar_tickfont_size   = 16,
                             colorbar_title_side      = "right",
                             colorbar_ypad            = 50,
                             colorbar_len             = 0.75)
        
                         
    fig_3d.update_layout(
             title_text                             = save_filename, 
             title_x                                = 0.5,
             width                                  = width,
             height                                 = height,
             font_size                              = 12,
             scene_aspectmode                       = "manual",
             scene_aspectratio                      = dict(x=1, y=1, z=0.5),      
             scene_zaxis_range                      = [min_alt,max_alt*3],
             scene                                  = dict(xaxis_title='Latitude [nmi]',
                                                           yaxis_title='Longitude [nmi]',
                                                           zaxis_title='Elevation [ft]',
                                                           xaxis = dict(
                                                                backgroundcolor=background_color,
                                                                gridcolor="white",
                                                                showbackground=True,
                                                                zerolinecolor=grid_color,),
                                                           yaxis = dict(
                                                               backgroundcolor=background_color,
                                                               gridcolor=grid_color,
                                                               showbackground=True,
                                                               zerolinecolor="white"),
                                                           zaxis = dict(
                                                               backgroundcolor=background_color,
                                                               gridcolor=grid_color,
                                                               showbackground=True,
                                                               zerolinecolor="white",),),
             scene_camera=dict(up    = dict(x=0, y=0, z=1),
                               center= dict(x=-0.05, y=0, z=-0.20),
                               eye   = dict(x=-1.0, y=-1.0, z=.4))   
    ) 
    if show_figure:
        fig_3d.show() 
    if save_figure:
        fig_3d.write_image(save_filename, file_type)

    return fig_3d       

def colorax(vmin, vmax):
    return dict(cmin=vmin, cmax=vmax)
 