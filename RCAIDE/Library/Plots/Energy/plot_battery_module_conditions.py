## @ingroup Library-Plots-Energy
# RCAIDE/Library/Plots/Energy/plot_battery_module_conditions.py
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
## @ingroup Library-Plots-Energy
def plot_battery_module_conditions(results,
                                  save_figure = False,
                                  show_legend = True,
                                  save_filename = "Battery_Module_Conditions_",
                                  file_type = ".png",
                                  width = 11, height = 7):
    """
    Creates a six-panel plot showing various battery module-level conditions throughout flight.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery module conditions
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag for displaying plot legend (default: True)
        
    save_filename : str, optional
        Base name of file for saved figure (default: "Battery_Module_Conditions_")
        
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
    The function creates a 3x2 subplot containing:
        1. State of Charge (SOC) vs time
        2. Module energy vs time
        3. Module current vs time
        4. Module power vs time
        5. Module voltage vs time
        6. Module temperature vs time
    
    Each segment is plotted with a different color from the inferno colormap.
    Different battery modules are distinguished by different markers.
    
    **Major Assumptions**
    
    * For identical battery modules, only the first module's data is plotted
    * Time is converted from seconds to minutes for plotting
    * Energy is converted to Watt-hours for display
    
    **Definitions**
    
    'SOC'
        State of Charge - the level of charge of the battery module relative to its capacity
    'Module Power'
        Total electrical power of the battery module
    'Module Energy'
        Cumulative energy stored/discharged by the battery module
    """  
    
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     

    fig = plt.figure(save_filename)
    fig.set_size_inches(width,height)
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))      
    axis_1 = plt.subplot(3,2,1)
    axis_2 = plt.subplot(3,2,2) 
    axis_3 = plt.subplot(3,2,3) 
    axis_4 = plt.subplot(3,2,4)
    axis_5 = plt.subplot(3,2,5) 
    axis_6 = plt.subplot(3,2,6)
     
    for network in results.segments[0].analyses.energy.vehicle.networks: 
        busses  = network.busses 
        for bus in busses:
            for b_i, battery in enumerate(bus.battery_modules):
                if b_i == 0 or bus.identical_battery_modules == False:
                    for i in range(len(results.segments)):  
                        time    = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min    
                        battery_conditions  = results.segments[i].conditions.energy[bus.tag].battery_modules[battery.tag]    
                        module_power          = battery_conditions.power[:,0]
                        module_energy         = battery_conditions.energy[:,0]
                        module_volts          = battery_conditions.voltage_under_load[:,0] 
                        module_current        = battery_conditions.current[:,0]
                        module_SOC            = battery_conditions.cell.state_of_charge[:,0]   
                        module_temperature    = battery_conditions.temperature[:,0]   
                    
                        if b_i == 0 and i ==0:                             
                            axis_1.plot(time, module_SOC, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width, label = battery.tag)
                        else:
                            axis_1.plot(time, module_SOC, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_1.set_ylabel(r'SOC')
                        axis_1.set_ylim([0,1.1])
                        set_axes(axis_1)     
                    
                        axis_2.plot(time, module_energy/Units.Wh, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_2.set_ylabel(r'Energy (W-hr)')
                        set_axes(axis_2) 
                    
                        axis_3.plot(time, module_current, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_3.set_ylabel(r'Current (A)')
                        set_axes(axis_3)  
                    
                        axis_4.plot(time, module_power, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_4.set_ylabel(r'Power (W)')
                        set_axes(axis_4)     
                    
                        axis_5.plot(time, module_volts, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width) 
                        axis_5.set_ylabel(r'Voltage (V)')
                        set_axes(axis_5) 
                    
                        axis_6.plot(time, module_temperature, color = line_colors[i], marker = ps.markers[b_i], linewidth = ps.line_width)
                        axis_6.set_ylabel(r'Temperature, $\degree$C')
                        set_axes(axis_6)  
                  
    if show_legend:      
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text   = 'Battery Module Conditions'       
    fig.suptitle(title_text) 
    
    if save_figure:
        plt.savefig(save_filename + battery.tag + file_type)    
    return fig 