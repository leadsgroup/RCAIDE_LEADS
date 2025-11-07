# RCAIDE/Library/Plots/Geometry/plot_rotor_geometry.py
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
def plot_rotor_geometry(prop,
                        face_color = 'red',
                        edge_color = 'black',
                        show_figure = True, 
                        save_figure = False,
                        save_filename = "Propeller_Geometry",
                        file_type = ".png",
                        width = 11, height = 7):    
    """
    Creates a 2D visualization of rotor/propeller geometry distributions.

    Parameters
    ----------
    prop : Propeller
        RCAIDE propeller/rotor data structure containing geometry information 
        
    show_figure : bool, optional
        Flag to display the figure (default: True)
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Propeller_Geometry")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")

    Returns
    -------
    fig : plotly.graph_objects.Figure

    Notes
    -----
    Creates a four-panel plot showing:
        1. Twist angle vs radial position
        2. Chord length vs radial position
        3. Maximum thickness vs radial position
        4. Mid-chord alignment vs radial position
    
    **Major Assumptions**
    
    * Distributions are defined at consistent radial stations
    * Twist is in degrees
    * Chord and thickness are in meters
    * Mid-chord alignment is in meters
    
    **Definitions**
    
    'Twist'
        Local blade angle relative to rotation plane
    'Chord'
        Local blade section length
    'Thickness'
        Maximum thickness of local blade section
    'Mid-chord Alignment'
        Offset of section mid-chord from reference line
    """
    # Unpack 
    r     = prop.radius_distribution 
    beta  = prop.twist_distribution/Units.degrees 
    c     = prop.chord_distribution 
    t_max = prop.max_thickness_distribution 
    MCA   = prop.mid_chord_alignment   
    
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    # get line colors for plots 
    line_color   = 'black'
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
                    
    axis_1 = plt.subplot(2,2,1) 
    axis_1.plot(r, c, color = line_color, marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width ) 
    axis_1.set_ylabel('Chord [m]') 
    axis_1.set_xlabel('Radial Station')       
    set_axes(axis_1)    

    axis_2 = plt.subplot(2,2,2)        
    axis_2.plot(r, beta, color = line_color, marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width) 
    axis_2.set_ylabel('Twist [deg]')
    axis_2.set_xlabel('Radial Station')
    set_axes(axis_2) 

    axis_3 = plt.subplot(2,2,3) 
    axis_3.plot(r, t_max, color =line_color, marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width)
    axis_3.set_xlabel('Radial Station')
    axis_3.set_ylabel('Max Thicness [t]')
    set_axes(axis_3) 

    axis_4 = plt.subplot(2,2,4)        
    axis_4.plot(r, MCA, color = line_color, marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width)
    axis_4.set_xlabel('Radial Station')
    axis_4.set_ylabel('Mid Chord Alignment') 
    set_axes(axis_4)  
    
    # Adjusting the sub-plots for legend
    fig.tight_layout() 
    fig.subplots_adjust(top=0.8)  
    
    if save_figure:
        fig.savefig(save_filename   + file_type)  
    return fig         

