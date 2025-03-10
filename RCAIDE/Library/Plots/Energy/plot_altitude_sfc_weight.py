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
def plot_altitude_sfc_weight(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Weight_Fuel_Consumption" ,
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
        Name of file for saved figure (default: "Altitude_SFC_Weight")
        
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
        2. Vehicle weight vs time
        1. Fuel consumption vs time
        3. Specific fuel consumption vs time
        4. Fuel consumption rate vs time
    
    Each segment is plotted with a different color from the inferno colormap.
  
    **Definitions**
    
    'SFC'
        Specific Fuel Consumption - measure of the fuel efficiency of an engine design 
        with respect to thrust output
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
    axis_4 = plt.subplot(2,2,4)
    
    for i in range(len(results.segments)): 
        time      = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        Weight    = results.segments[i].conditions.weights.total_mass[:, 0] * 9.81   
        mdot      = results.segments[i].conditions.weights.vehicle_mass_rate[:, 0]
        thrust    = results.segments[i].conditions.frames.body.thrust_force_vector[:, 0]
        thrust    = results.segments[i].conditions.frames.body.thrust_force_vector[:, 0]
        fuel_mass = results.segments[i].conditions.energy.cumulative_fuel_consumption[:, 0]
        sfc       = (mdot / Units.lb) / (thrust / Units.lbf) * Units.hr
        

        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')       
        axis_1.plot(time, Weight/1000 , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name )  
        axis_1.set_ylabel(r'Weight (kN)')  
        set_axes(axis_1) 

        axis_2.plot(time, fuel_mass, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_2.set_xlabel('Time (mins)')
        axis_2.set_ylabel(r'Fuel Consumption (kg)')
        set_axes(axis_2)
        

        axis_3.plot(time, sfc, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_3.set_xlabel('Time (mins)')
        axis_3.set_ylabel(r'SFC (lb/lbf-hr)')
        set_axes(axis_3) 

        axis_4.plot(time, mdot, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_4.set_xlabel('Time (mins)')
        axis_4.set_ylabel(r'Fuel Rate (kg/s)')
        set_axes(axis_4)         
        
    
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Weight and Fuel Consumption'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 