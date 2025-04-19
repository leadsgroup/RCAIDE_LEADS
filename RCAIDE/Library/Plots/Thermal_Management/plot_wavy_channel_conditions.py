# RCAIDE/Library/Plots/Thermal_Management/plot_wavy_channel_conditions.py
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
#   plot_wavy_channel_conditions
# ----------------------------------------------------------------------------------------------------------------------   
def plot_wavy_channel_conditions(wavy_channel, results, coolant_line,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Wavy_Channel_Conditions",
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates a multi-panel visualization of wavy channel heat exchanger performance.

    Parameters
    ----------
    wavy_channel : Component
        Wavy channel heat exchanger component containing:
            - tag : str
                Unique identifier for the heat exchanger
            
    results : Results
        RCAIDE results data structure containing:
            - segments[i].conditions.frames.inertial.time[:,0]
                Time history for each segment
            - segments[i].conditions.energy[coolant_line.tag][wavy_channel.tag]
                Heat exchanger data containing:
                    - outlet_coolant_temperature[:,0]
                        Coolant outlet temperature in K
                    - coolant_mass_flow_rate[:,0]
                        Coolant flow rate in kg/s
                    - power[:,0]
                        Heat transfer rate in watts
                    
    coolant_line : Component
        Coolant line component containing:
            - tag : str
                Unique identifier for the coolant circuit
            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Wavy_Channel_Conditions")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Handle to the generated figure containing three subplots
        
    Notes
    -----
    Creates visualization showing:
        * Thermal performance metrics
        * Flow conditions
        * Heat transfer characteristics
        * Time history for each segment
    
    **Definitions**
    
    'Wavy Channel'
        Heat exchanger with sinusoidal flow paths
    'Hydraulic Diameter'
        4 × (flow area) / (wetted perimeter)
    'Heat Transfer Rate'
        Rate of thermal energy transfer
    'Mass Flow Rate'
        Mass of fluid flowing per unit time
    
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
    
    fig = plt.figure('Identical_'+ save_filename)
    fig.set_size_inches(width,height) 
    axis_1 = plt.subplot(2,2,1)
    axis_2 = plt.subplot(2,2,2) 
    axis_3 = plt.subplot(2,2,3)

    for network in results.segments[0].analyses.energy.vehicle.networks: 
        busses  = network.busses 
        for bus in busses:
            for b_i, battery in enumerate(bus.battery_modules):
                if b_i == 0 or bus.identical_battery_modules == False:
                    for i in range(len(results.segments)): 
                        time                            = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min    
                        wavy_channel_conditions         = results.segments[i].conditions.energy[coolant_line.tag][wavy_channel.tag]   
                        outlet_coolant_temperature      = wavy_channel_conditions.outlet_coolant_temperature[:,0]
                        coolant_mass_flow_rate          = wavy_channel_conditions.coolant_mass_flow_rate[:,0]
                        power                           = wavy_channel_conditions.power[:,0]       
                
                        if i == 0:                
                            axis_1.plot(time, outlet_coolant_temperature, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width, label = wavy_channel.tag)
                        else:
                            axis_1.plot(time, outlet_coolant_temperature, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_1.set_ylabel(r'Coolant Temp. (K)') 
                        set_axes(axis_1)     
                         
                        axis_2.plot(time, coolant_mass_flow_rate, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_2.set_ylabel(r'Coolant $\dot{m}$ (kg/s)')
                        set_axes(axis_2) 
                 
                        axis_3.plot(time, power, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_3.set_ylabel(r'HAS Power (W)')
                        axis_3.set_xlabel(r'Time (mins)')
                        set_axes(axis_3)
        
    if show_legend:          
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})     
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text   = 'Wavy_Channel_Properties'       
    fig.suptitle(title_text) 
    
    if save_figure:
        plt.savefig(wavy_channel.tag + file_type)    
    return fig 