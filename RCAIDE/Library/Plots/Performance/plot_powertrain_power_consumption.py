# RCAIDE/Library/Plots/Performance/plot_powertrain_power_consumption.py
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
def plot_powertrain_power_consumption(results,
                                      save_figure = False,
                                      show_legend = True,
                                      save_filename = "Powertain_Power_Conditions",
                                      file_type = ".png",
                                      width = 11, height = 7):
    """
    Creates a six-panel plot showing various powertrain component power consumption.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and powertrain conditions
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag for displaying plot legend (default: True)
        
    save_filename : str, optional
        Base name of file for saved figure (default: "Powertain_Power_Conditions")
        
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
    The function creates a 3x2 subplot containing:
        1. propulsive power vs time
        2. chemical  energy vs time
        3. electrical current vs time
        4. thermal  power vs time
        5. hydaulic power vstime
        6. pneumatic power vs time
    
    Each segment is plotted with a different color from the inferno colormap. 
    
    **Definitions** 
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
    axis_1 = plt.subplot(3,2,1) # propulsive
    axis_2 = plt.subplot(3,2,2) # chemical 
    axis_3 = plt.subplot(3,2,3) # electrical
    axis_4 = plt.subplot(3,2,4) # thermal 
    axis_5 = plt.subplot(3,2,5) # hydaulic 
    axis_6 = plt.subplot(3,2,6) # pneumatic
     

    for network in results.segments[0].analyses.vehicle.networks:        
        for i in range(len(results.segments)): 
            
            time                = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
            energy_conditions   = results.segments[i].state.conditions.energy
            
            compoments_names = ['converters', 'propulsors', 'modulators', 'sources', 'distributors', 'systems']
            
            for converter in network.converters: 
                if converter.propulsor_integrated != True: 
                    for power_type in  energy_conditions.converters[converter].inputs.power.keys():
                        power = -energy_conditions.converters[converter].inputs.power[power_type][:,0]
                        if power_type == "propulsive": 
                            axis_1.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_1.set_ylabel(r'$P_{Propulsive}$')
                            set_axes(axis_1)                           
                            
                        if power_type == "chemical":
                            axis_2.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_2.set_ylabel(r'$P_{Chemical}$')
                            set_axes(axis_2)       
                            
                        if power_type == "electrical":
                            axis_3.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_3.set_ylabel(r'$P_{Electrical}$')
                            set_axes(axis_3)       
                            
                        if power_type == "thermal":
                            axis_4.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_4.set_ylabel(r'$P_{Thermal}$')
                            set_axes(axis_4)       
                            
                        if power_type == "hydaulic":
                            axis_5.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_5.set_ylabel(r'$P_{Hydraulic}$')
                            set_axes(axis_5)       
                            
                        if power_type == "pneumatic": 
                            axis_6.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_6.set_ylabel(r'$P_{Pneumatic}$')
                            set_axes(axis_6)
                            
                
                    for power_type in  energy_conditions.converters[converter].outputs.power.keys():
                        power = energy_conditions.converters[converter].outputs.power[power_type][:,0]
                        if power_type == "propulsive": 
                            axis_1.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_1.set_ylabel(r'$P_{Propulsive}$')
                            set_axes(axis_1)                           
                            
                        if power_type == "chemical":
                            axis_2.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_2.set_ylabel(r'$P_{Chemical}$')
                            set_axes(axis_2)       
                            
                        if power_type == "electrical":
                            axis_3.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_3.set_ylabel(r'$P_{Electrical}$')
                            set_axes(axis_3)       
                            
                        if power_type == "thermal":
                            axis_4.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_4.set_ylabel(r'$P_{Thermal}$')
                            set_axes(axis_4)       
                            
                        if power_type == "hydaulic":
                            axis_5.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_5.set_ylabel(r'$P_{Hydraulic}$')
                            set_axes(axis_5)       
                            
                        if power_type == "pneumatic": 
                            axis_6.plot(time, power, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
                            axis_6.set_ylabel(r'$P_{Pneumatic}$')
                            set_axes(axis_6)                
                
    
            for propulsor in network.propulsors:
                
    
            for modulator in network.modulators:
                
    
            for sources in network.sources:
                
    
            for distributor in network.distributors:                      
                
            for c_i, component_class in enumerate(compoments_names):
                
                    
               
               
                            
                  
    if show_legend:      
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)   
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text   = 'Powertrain Power Conditions'       
    fig.suptitle(title_text) 
    
    if save_figure:
        plt.savefig(save_filename + file_type)    
    return fig  