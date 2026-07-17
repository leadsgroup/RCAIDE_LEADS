# RCAIDE/Library/Plots/Performance/Stability/plot_control_surface_conditions.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style, segment_colors
import matplotlib.pyplot as plt
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_control_surface_conditions(results,
                           save_figure = False,
                           show_legend = True,
                           save_filename = "Control Surface Conditions",
                           file_type = ".png",
                           width = 11, height = 7):
    """
    
    """
  
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    # get line colors for plots 
    line_colors   = segment_colors(len(results.segments))     
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
    axis_1 = fig.add_subplot(1,1,1)   
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min  
         
        cs_i = 0
        for cs_key in results.segments[i].conditions.control_surfaces.keys(): 
            if i == 0: 
                cs_deflection =  results.segments[i].conditions.control_surfaces[cs_key].deflection[:,0] / Units.deg
                axis_1.plot(time, cs_deflection, color = line_colors[i], marker = ps.markers[cs_i], linewidth = ps.line_width, label = cs_key ) 
            else: 
                cs_deflection =  results.segments[i].conditions.control_surfaces[cs_key].deflection[:,0] / Units.deg
                axis_1.plot(time, cs_deflection, color = line_colors[i], marker = ps.markers[cs_i], linewidth = ps.line_width)
            cs_i += 1
         
    
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel(r'Control Surface Deflection (deg)')
    axis_1.set_ylim([-40, 40])
    set_axes(axis_1)
    
    if show_legend:        
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Control Surface', prop={'size': ps.legend_font_size, 'weight': 'heavy'}) 
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.75)
    
    # set title of plot 
    title_text    = 'Control Surface Deflection'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return  fig 