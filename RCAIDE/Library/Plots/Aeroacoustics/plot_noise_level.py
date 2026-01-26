# RCAIDE/Library/Plots/Noise/plot_noise_level.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style 

import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ---------------------------------------------------------------------------------------------------------------------- 
def plot_noise_level(noise_data,
                     noise_level = False,
                     save_figure = False,
                     save_filename = "Sideline_Noise_Levels",
                     file_type = ".png",
                     width = 11, height = 7):
    """
    Creates a visualization of A-weighted Sound Pressure Levels at various sideline distances.

    Parameters
    ----------
    noise_data : NoiseData
        RCAIDE noise data structure containing:
            - microphone_y_resolution : int
                Number of sideline measurement positions
            - microphone_locations : ndarray
                3D array of microphone positions where:
                    - [:,:,0] : x-positions (longitudinal)
                    - [:,:,1] : y-positions (sideline)
                    - [:,:,2] : z-positions (vertical)
            
    noise_level : ndarray, optional
        2D array of noise levels at measurement points (default: False)
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Sideline_Noise_Levels")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Handle to the generated figure showing noise levels vs range

    Notes
    -----
    Creates visualization showing:
        * Noise level variation with distance
        * Multiple sideline measurement positions
        * Color-coded sideline distances
        * Customizable appearance
    
    **Major Assumptions**
        * Noise levels are A-weighted SPL in dBA
        * Microphones are in a regular grid
        * Range is in nautical miles
        * Sideline distances are in meters
    
    **Definitions**
    
    'Sound Pressure Level'
        A-weighted acoustic pressure level in dBA
    'Sideline Distance'
        Perpendicular distance from flight path
    'Range'
        Distance along flight path
    
    See Also
    --------
    RCAIDE.Library.Plots.Noise.plot_2D_noise_contour : 2D contour visualization
    RCAIDE.Library.Plots.Noise.plot_3D_noise_contour : 3D noise field visualization
    """      
    N_gm_y       = noise_data.microphone_y_resolution 
    gm           = noise_data.microphone_locations    
    gm_x         = gm[:,:,0]
    gm_y         = gm[:,:,1]    
    

    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters) 
      
    fig         = plt.figure(save_filename)
    fig.set_size_inches(width,height)
    axes        = fig.add_subplot(1,1,1) 
    
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,N_gm_y))  
      
    for k in range(N_gm_y):    
        axes.plot(gm_x[:,0]/Units.nmi, noise_level[:,k], marker = 'o', color = line_colors[k], label= r'mic at y = ' + str(round(gm_y[0,k],1)) + r' m' ) 
    axes.set_ylabel('SPL [dBA]')
    axes.set_xlabel('Range [nmi]')  
    set_axes(axes)
    axes.legend(loc='upper right')         
    if save_figure:
        plt.savefig(save_filename + ".png")    
        
    return fig