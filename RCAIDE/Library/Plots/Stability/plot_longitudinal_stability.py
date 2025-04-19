# RCAIDE/Library/Plots/Performance/Stability/plot_longitudinal_stability.py
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
def plot_longitudinal_stability(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Longitudinal_Stability",
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates a multi-panel visualization of longitudinal stability characteristics.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - segments[i].conditions.frames.inertial.time[:,0]
                Time history for each segment
            - segments[i].conditions.aerodynamics.angles.theta[:,0]
                Pitch angle history
            - segments[i].conditions.aerodynamics.angles.alpha[:,0]
                Angle of attack history
            - segments[i].conditions.control_surfaces.elevator.deflection[:,0]
                Elevator deflection history
            - segments[i].tag
                Name/identifier of each segment
            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Longitudinal_Stability")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure

    Notes
    -----
    Creates visualization showing:
        * Pitch attitude response
        * Aerodynamic angle evolution
        * Longitudinal control inputs
        * Trajectory angle history
        * Time history for each segment
    
    **Major Assumptions**
        * Angles are in degrees
        * Time is in minutes
        * Positive deflections follow right-hand rule
   
    **Definitions**
    
    'Pitch Angle'
        Nose-up/down attitude relative to horizon
    'Angle of Attack'
        Angle between velocity vector and body reference line
    'Flight Path Angle'
        Angle between velocity vector and horizon
    'Elevator Deflection'
        Pitch control surface position
    
    See Also
    --------
    RCAIDE.Library.Plots.Stability.plot_lateral_stability : Lateral-directional stability analysis
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
    

    axis_1 = plt.subplot(3,2,1)    
    axis_2 = plt.subplot(3,2,2) 
    axis_3 = plt.subplot(3,2,3)  
    axis_4 = plt.subplot(3,2,4)  
    axis_5 = plt.subplot(3,2,5)
    axis_6 = plt.subplot(3,2,6) 
    
    for i in range(len(results.segments)): 
        time       = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        c_m        = results.segments[i].conditions.static_stability.coefficients.M[:,0]   
        SM         = results.segments[i].conditions.static_stability.static_margin[:,0]  
        delta_e    = results.segments[i].conditions.control_surfaces.elevator.deflection[:,0] / Units.deg
        CM_delta_e = results.segments[i].conditions.static_stability.derivatives.CM_delta_e[:,0]
        Cm_alpha   = results.segments[i].conditions.static_stability.derivatives.CM_alpha[:,0]
        CL_alpha   = results.segments[i].conditions.static_stability.derivatives.Clift_alpha[:,0] 
          
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')  
        
        axis_1.plot(time, c_m, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name) 
        axis_1.set_ylabel(r'$C_M$')
        axis_1.set_ylim([-1, 1])  
        set_axes(axis_1) 

        axis_2.plot(time, Cm_alpha, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_2.set_ylabel(r'$C_M\alpha$')
        axis_2.set_ylim([-1, 1]) 
        set_axes(axis_2) 
        
        axis_3.plot(time,SM , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_3.set_xlabel('Time (mins)')
        axis_3.set_ylabel(r'Static Margin (%)')
        set_axes(axis_3)  

        axis_4.plot(time,delta_e , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_4.set_xlabel('Time (mins)')
        axis_4.set_ylabel(r'Elevator Defl.n') 
        axis_4.set_ylim([-15, 15]) 
        set_axes(axis_4) 
        
        axis_5.plot(time,CM_delta_e , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_5.set_xlabel('Time (mins)')
        axis_5.set_ylabel(r'$C_M\delta_e$')
        axis_5.set_ylim([-1, 1]) 
        set_axes(axis_5)
        
        axis_6.plot(time,CL_alpha, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_6.set_xlabel('Time (mins)')
        axis_6.set_ylabel(r'$C_L\alpha$')
        axis_6.set_ylim([-1, 10]) 
        set_axes(axis_6)    
        
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
 