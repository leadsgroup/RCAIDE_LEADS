# RCAIDE/Library/Plots/Performance/Stability/plot_lateral_stability.py
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
def plot_lateral_stability(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Lateral_Stability",
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates a multi-panel visualization of lateral-directional stability characteristics.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - segments[i].conditions.frames.inertial.time[:,0]
                Time history for each segment
            - segments[i].conditions.aerodynamics.angles.phi[:,0]
                Bank angle history
            - segments[i].conditions.control_surfaces.aileron.deflection[:,0]
                Aileron deflection history
            - segments[i].conditions.control_surfaces.rudder.deflection[:,0]
                Rudder deflection history
            - segments[i].tag
                Name/identifier of each segment
            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Lateral_Stability")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Handle to the generated figure containing three subplots:
            * Bank angle vs time
            * Aileron deflection vs time
            * Rudder deflection vs time

    Notes
    -----
    Creates visualization showing:
        * Roll attitude response
        * Lateral control inputs
        * Directional control inputs
        * Time history for each segment
    
    **Major Assumptions**
        * Angles are in degrees
        * Time is in minutes
        * Positive deflections follow right-hand rule
    
    **Definitions**
    
    'Bank Angle'
        Roll attitude relative to horizon
    'Aileron Deflection'
        Roll control surface position
    'Rudder Deflection'
        Yaw control surface position
    
    See Also
    --------
    RCAIDE.Library.Plots.Stability.plot_longitudinal_stability : Longitudinal stability analysis
    RCAIDE.Library.Plots.Stability.plot_flight_forces_and_moments : Force/moment visualization
    """  

    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))     
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
    axis_1 = plt.subplot(2,2,1) 
    axis_2 = plt.subplot(2,2,2)  
    axis_3 = plt.subplot(2,2,3)    
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min  
        phi      = results.segments[i].conditions.aerodynamics.angles.phi[:,0] / Units.deg          
        delta_a  = results.segments[i].conditions.control_surfaces.aileron.deflection[:,0] / Units.deg  
        delta_r  = results.segments[i].conditions.control_surfaces.rudder.deflection[:,0] / Units.deg   
          
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')
        
        axis_1.plot(time, phi, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name)
        axis_1.set_ylabel(r'$Bank Angle \phi$') 
        set_axes(axis_1)     

        axis_2.plot(time,delta_a , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_2.set_xlabel('Time (mins)')
        axis_2.set_ylabel(r'Aileron Defl. (deg)')
        set_axes(axis_2)  

        axis_3.plot(time,delta_r , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_3.set_xlabel('Time (mins)')
        axis_3.set_ylabel(r'Rudder Defl. (deg)')
        set_axes(axis_3)         
         
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text    = 'Stability Coefficents'      
    fig.suptitle(title_text)
 
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 