# RCAIDE/Library/Plots/Performance/plot_fuel_tank_conditions.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_fuel_tank_conditions(results,
                                  save_figure = False,
                                  show_legend = True,
                                  save_filename = "Fuel_Tank_Conditions_",
                                  file_type = ".png",
                                  width = 11, height = 7):
    """
    Creates a two-panel plot showing various fuel tank conditions throughout flight. 
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
    axis_1 = plt.subplot(2,2,1)
    axis_2 = plt.subplot(2,2,2) 
    axis_3 = plt.subplot(2,2,3) 
    axis_4 = plt.subplot(2,2,4) 
     
    for network in results.segments[0].analyses.vehicle.networks: 
        for  t_i,  source in enumerate(network.sources): 
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank): 
                for i in range(len(results.segments)):  
                    time    = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min    
                    tank_conditions    = results.segments[i].conditions.energy.sources[source.tag] 
                    fsr                = tank_conditions.power_split_ratio[:,0] 
                    m_dot              = tank_conditions.mass_flow_rate[:,0]
                    sm_dot             = tank_conditions.secondary_mass_flow_rate[:,0] 
                    tank_mass          = results.segments[i].conditions.weights.components.mass[source.tag]
                    fuel_mass          = results.segments[i].conditions.weights.components.mass[source.fuel.tag]
                    total_mass         = tank_mass + fuel_mass   
                
                    if i ==0:                             
                        axis_1.plot(time, total_mass, color = line_colors[i], marker = ps.markers[t_i], linewidth = ps.line_width, label = fuel_tank.tag)
                    else:
                        axis_1.plot(time, total_mass, color = line_colors[i], marker = ps.markers[t_i], linewidth = ps.line_width)
                    axis_1.set_ylabel(r'Tank+Fuel Mass [kg]') 
                    set_axes(axis_1)     
                
                    axis_2.plot(time, m_dot, color = line_colors[i], marker = ps.markers[t_i], linewidth = ps.line_width)
                    axis_2.set_ylabel(r'Flow Rate [kg/s]')
                    set_axes(axis_2)

                    axis_3.plot(time, fsr, color = line_colors[i], marker = ps.markers[t_i], linewidth = ps.line_width)
                    axis_3.set_ylabel(r'Fuel Split')
                    axis_3.set_xlabel('Time (mins)')  
                    set_axes(axis_3) 

                    axis_4.plot(time, sm_dot, color = line_colors[i], marker = ps.markers[t_i], linewidth = ps.line_width)
                    axis_4.set_ylabel(r'Secondary Flow Rate [kg/s]')
                    axis_4.set_xlabel('Time (mins)')  
                    set_axes(axis_4)                      
                 
                  
    if show_legend:      
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text   = 'Fuel Tank Conditions'       
    fig.suptitle(title_text) 
    
    if save_figure:
        plt.savefig(save_filename + file_type)    
    return fig 