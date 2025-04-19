# RCAIDE/Library/Plots/Thermal_Management/plot_air_cooled_conditions.py
# 
# 
# Created:  Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#   plot_air_cooled_conditions
# ----------------------------------------------------------------------------------------------------------------------   
def plot_air_cooled_conditions(air_cooled, results, coolant_line, 
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Air_Cooled_Conditions",
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates a multi-panel visualization of air-cooled heat exchanger performance.

    Parameters
    ----------
    air_cooled : Component
        Air-cooled heat exchanger component containing:
            * tag : str
                Unique identifier for the heat exchanger
            
    results : Results
        RCAIDE results data structure containing:
            * segments[i].conditions.frames.inertial.time[:,0]
                Time history for each segment
            * segments[i].conditions.energy[coolant_line.tag][air_cooled.tag]
                Heat exchanger performance data containing:
                    * effectiveness[:,0]
                        Heat exchanger effectiveness
                    * total_heat_removed[:,0]
                        Total heat transfer rate
                    
    coolant_line : Component
        Coolant line component containing:
            * tag : str
                Unique identifier for the coolant circuit
            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Air_Cooled_Conditions")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Handle to the generated figure
   
    Notes
    -----
    Creates visualization showing:
        * Heat exchanger performance metrics
        * Thermal effectiveness evolution
        * Heat transfer rate history
        * Time history for each segment
    
    **Definitions**
    
    'Effectiveness'
        Ratio of actual to maximum possible heat transfer
    'Heat Transfer Rate'
        Rate of thermal energy transfer between fluids
    'Heat Capacity Rate'
        Product of mass flow rate and specific heat
    
    See Also
    --------
    RCAIDE.Library.Plots.Thermal_Management.plot_thermal_management_performance : Overall system performance
    RCAIDE.Library.Plots.Thermal_Management.plot_cross_flow_heat_exchanger_conditions : Cross-flow heat exchanger analysis
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

    fig = plt.figure(save_filename)
    fig.set_size_inches(width,height)  
    axis_1 = plt.subplot(2,1,1)
    axis_2 = plt.subplot(2,1,2)
    

 
    for network in results.segments[0].analyses.energy.vehicle.networks: 
        busses  = network.busses 
        for bus in busses:
            for b_i, battery in enumerate(bus.battery_modules):
                if b_i == 0 or bus.identical_battery_modules == False:
                    for i in range(len(results.segments)): 
                        time                       = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min    
                        air_cooled_conditions      = results.segments[i].conditions.energy[coolant_line.tag][air_cooled.tag]   
                        effectiveness              = air_cooled_conditions.effectiveness[:,0]
                        total_heat_removed         = air_cooled_conditions.total_heat_removed[:,0] 
                        
                        if i == 0: 
                            axis_1.plot(time, effectiveness, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width, label = battery.tag)
                        else:
                            axis_1.plot(time, effectiveness, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_1.set_ylabel(r'Effectiveness') 
                        set_axes(axis_1)     
                         
                        axis_2.plot(time, total_heat_removed, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_2.set_ylabel(r'Total Heat Removed (W)')
                        set_axes(axis_2)  
            
    if show_legend:    
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})  
        
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text   = 'Air_Cooled_Properties'       
    fig.suptitle(title_text) 
    
    if save_figure:
        plt.savefig(save_filename + air_cooled.tag + file_type)    
    return fig     