# RCAIDE/Library/Plots/Energy/plot_battery_degradation.py
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
def plot_battery_degradation(results,
                            save_figure = False,
                            line_color = 'bo-',
                            line_color2 = 'rs--',
                            save_filename = "Battery_Degradation",
                            file_type = ".png",
                            width = 11, height = 7):
    """
    Creates a six-panel plot showing battery degradation metrics against various parameters.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery degradation metrics
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    line_color : str, optional
        Matplotlib format string for first line style (default: 'bo-')
        
    line_color2 : str, optional
        Matplotlib format string for second line style (default: 'rs--')
        
    save_filename : str, optional
        Base name of file for saved figure (default: "Battery_Degradation")
        
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
    The function creates a 3x2 subplot showing:
        Left column (Capacity fade vs):
            1. Charge throughput (Ah)
            2. Time (hours)
            3. Time (days)

        Right column (Resistance growth vs):
            1. Charge throughput (Ah)
            2. Time (hours)
            3. Time (days)
    
    **Definitions**
    
    'Capacity Fade'
        The loss of energy storage capacity over time/usage
    'Resistance Growth'
        The increase in internal resistance over time/usage
    'Charge Throughput'
        The cumulative amount of charge that has passed through the battery
    """ 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    

    for network in results.segments[0].analyses.energy.vehicle.networks: 
        busses  = network.busses
        for bus in busses:
            if bus.identical_battery_modules:
                for i, battery in enumerate(bus.battery_modules):
                    if i == 0:
                        fig = plt.figure(save_filename + '_' + battery.tag)
                        fig.set_size_inches(width,height)  
                        num_segs          = len(results.segments)
                        time_hrs          = np.zeros(num_segs)  
                        capacity_fade     = np.zeros_like(time_hrs)
                        resistance_growth = np.zeros_like(time_hrs)
                        cycle_day         = np.zeros_like(time_hrs)
                        charge_throughput = np.zeros_like(time_hrs)    
                             
                        for i in range(len(results.segments)): 
                            time_hrs[i]    = results.segments[i].conditions.frames.inertial.time[-1,0]  / Units.hour   
                            battery_conditions  = results.segments[i].conditions.energy.busses[bus.tag].battery_modules[battery.tag]
                            cycle_day[i]          = battery_conditions.cell.cycle_in_day
                            capacity_fade[i]      = battery_conditions.cell.capacity_fade_factor
                            resistance_growth[i]  = battery_conditions.cell.resistance_growth_factor
                            charge_throughput[i]  = battery_conditions.cell.charge_throughput[-1,0]  
                 
                        axis_1 = plt.subplot(3,2,1)
                        axis_1.plot(charge_throughput, capacity_fade, color = ps.color , marker = ps.markers[0], linewidth = ps.line_width ) 
                        axis_1.set_ylabel('$E/E_0$')
                        axis_1.set_xlabel('Ah')
                        set_axes(axis_1)      
                    
                        axis_2 = plt.subplot(3,2,3)
                        axis_2.plot(time_hrs, capacity_fade, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width ) 
                        axis_2.set_ylabel('$E/E_0$')
                        axis_2.set_xlabel('Time (hrs)')
                        set_axes(axis_2)     
                    
                        axis_3 = plt.subplot(3,2,5)
                        axis_3.plot(cycle_day, capacity_fade, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width ) 
                        axis_3.set_ylabel('$E/E_0$')
                        axis_3.set_xlabel('Time (days)')
                        set_axes(axis_3)     
                    
                        axis_4 = plt.subplot(3,2,2) 
                        axis_4.plot(charge_throughput, resistance_growth, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width )
                        axis_4.set_ylabel('$R/R_0$')
                        axis_4.set_xlabel('Ah')
                        set_axes(axis_4)      
                    
                        axis_5 = plt.subplot(3,2,4) 
                        axis_5.plot(time_hrs, resistance_growth, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width )
                        axis_5.set_ylabel('$R/R_0$')
                        axis_5.set_xlabel('Time (hrs)')
                        set_axes(axis_5)     
                    
                        axis_6 = plt.subplot(3,2,6) 
                        axis_6.plot(cycle_day, resistance_growth, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width )
                        axis_6.set_ylabel('$R/R_0$')
                        axis_6.set_xlabel('Time (days)')
                        set_axes(axis_6)              
    
                        
    # set title of plot 
    title_text    = 'Battery Cell Degradation: ' + battery.tag   
    fig.suptitle(title_text) 
    
    plt.tight_layout()    
    if save_figure:    
        fig.savefig(save_filename + '_'+ battery.tag + file_type) 

    return fig 

