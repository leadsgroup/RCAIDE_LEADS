# RCAIDE/Library/Plots/Mass_Properties/plot_center_of_gravity_drift.py
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
def plot_center_of_gravity_drift(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Center_of_Gravity" ,
                             file_type = ".png", 
                             width = 11, height = 7):
    """
    Creates a four-panel plot showing throttle settings, vehicle weight, specific fuel consumption (SFC), 
    and fuel consumption rate over time.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag for displaying plot legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Center_of_Gravity")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure handle containing the generated plots

    Notes
    -----
    The function creates a 2x2 subplot containing:
        1. Vehicle center of gravity x-location vs time
        2. Vehicle center of gravity y-location vs time
        3. Vehicle center of gravity z-location vs time 
    
    Each segment is plotted with a different color from the inferno colormap.
   
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
        time      = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min 
        CG_x      = results.segments[i].conditions.weights.vehicle.global_center_of_gravity[:, 0]  
        CG_y      = results.segments[i].conditions.weights.vehicle.global_center_of_gravity[:, 1]  
        CG_z      = results.segments[i].conditions.weights.vehicle.global_center_of_gravity[:, 2]  
        
        axis_1.set_ylabel(r'$C.G._x$ (m)')  
        axis_2.set_ylabel(r'$C.G._y$ (m)')
        axis_3.set_ylabel(r'$C.G._z$ (m)')   

        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')       
        axis_1.plot(time, CG_x, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name )    
        set_axes(axis_1) 

        axis_2.plot(time, CG_y, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_2.set_xlabel('Time (mins)')
        set_axes(axis_2) 

        axis_3.plot(time, CG_z, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_3.set_xlabel('Time (mins)')
        set_axes(axis_3) 
 
        
    
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Center of Gravity'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 