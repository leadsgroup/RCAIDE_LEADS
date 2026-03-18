# RCAIDE/Library/Plots/Mass_Properties/plot_moment_of_intertia_drift.py
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
def plot_moment_of_intertia_drift(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Moment_of_Inertia" ,
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
        Name of file for saved figure (default: "Moment_of_Inertia")
        
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
        1. I_xx vs time
        2. I_yy vs time
        3. I_zz vs time
    
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
    axis_1 = plt.subplot(3,3,1)
    axis_2 = plt.subplot(3,3,2)
    axis_3 = plt.subplot(3,3,3)
    axis_4 = plt.subplot(3,3,4)
    axis_5 = plt.subplot(3,3,5)
    axis_6 = plt.subplot(3,3,6)
    axis_7 = plt.subplot(3,3,7)
    axis_8 = plt.subplot(3,3,8)
    axis_9 = plt.subplot(3,3,9)
    
    for i in range(len(results.segments)): 
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min   
        I_xx = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Ixx[:,0]    
        I_xy = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Ixy[:,0]    
        I_xz = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Ixz[:,0]    
        I_yx = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Iyx[:,0]    
        I_yy = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Iyy[:,0]    
        I_yz = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Iyz[:,0]    
        I_zx = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Izx[:,0]    
        I_zy = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Izy[:,0]    
        I_zz = results.segments[i].conditions.weights.vehicle.moments_of_inertia_Izz[:,0]            
        
      
        axis_1.set_ylabel(r'$I_{xx}$')
        axis_2.set_ylabel(r'$I_{xy}$')
        axis_3.set_ylabel(r'$I_{xz}$')   
        axis_4.set_ylabel(r'$I_{yx}$')
        axis_5.set_ylabel(r'$I_{yy}$')
        axis_6.set_ylabel(r'$I_{yz}$')   
        axis_7.set_ylabel(r'$I_{zx}$')
        axis_8.set_ylabel(r'$I_{zy}$')
        axis_9.set_ylabel(r'$I_{zz}$')   

        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')       
        axis_1.plot(time, I_xx, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name )    
        axis_2.plot(time, I_xy, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_3.plot(time, I_xz, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_4.plot(time, I_yx, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_5.plot(time, I_yy, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_6.plot(time, I_yz, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_7.plot(time, I_zx, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_8.plot(time, I_zy, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_9.plot(time, I_zz, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)  
        
        axis_1.set_xlabel('Time (mins)')
        axis_2.set_xlabel('Time (mins)')
        axis_3.set_xlabel('Time (mins)')
        axis_4.set_xlabel('Time (mins)')
        axis_5.set_xlabel('Time (mins)')
        axis_6.set_xlabel('Time (mins)')
        axis_7.set_xlabel('Time (mins)')
        axis_8.set_xlabel('Time (mins)')
        axis_9.set_xlabel('Time (mins)') 

        set_axes(axis_1) 
        set_axes(axis_2) 
        set_axes(axis_3) 
        set_axes(axis_4) 
        set_axes(axis_5) 
        set_axes(axis_6)   
        set_axes(axis_7) 
        set_axes(axis_8) 
        set_axes(axis_9)                   
        
    
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Moment of Intertia'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 