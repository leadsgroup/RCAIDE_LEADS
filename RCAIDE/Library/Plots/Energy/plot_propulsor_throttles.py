## @ingroup Library-Plots-Energy
# RCAIDE/Library/Plots/Energy/plot_altitude_sfc_weight.py
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
## @ingroup Library-Plots-Performance-Energy-Fuel
def plot_propulsor_throttles(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Propulsor_Throttles" ,
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates a plot showing throttle settings for all propulsors throughout the flight mission.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and propulsor throttle settings
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag for displaying plot legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Propulsor_Throttles")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure handle containing the generated plot

    Notes
    -----
    The function creates a single plot showing:
        - Throttle settings vs time for each propulsor
        - Different flight segments distinguished by different colors
        - Different propulsors distinguished in the legend
    
    **Major Assumptions**
    
    * Throttle values are normalized between 0 and 1
    * Time is converted from seconds to minutes for plotting
    * All propulsors have throttle data available
    
    **Definitions**
    
    'Throttle'
        Normalized control input that determines propulsor power setting
    'Flight Segment'
        Distinct phase of the mission with specific throttle requirements
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
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min  
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ') 
        
        # power 
        axis_1 = plt.subplot(1,1,1)
        axis_1.set_ylabel(r'Throttle')
        set_axes(axis_1)               
        for network in results.segments[i].analyses.energy.vehicle.networks: 
            for j ,  propulsor in enumerate(network.propulsors):
                eta = results.segments[i].conditions.energy[propulsor.tag].throttle[:,0]  
                if j == 0 and i ==0:               
                    axis_1.plot(time, eta, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name + ': '+ propulsor.tag )
                else:
                    axis_1.plot(time, eta, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)     
    
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Propulsor', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Throttle'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 