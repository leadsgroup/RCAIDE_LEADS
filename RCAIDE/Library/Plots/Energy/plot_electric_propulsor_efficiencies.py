## @ingroup Library-Plots-Energy
# RCAIDE/Library/Plots/Energy/plot_electric_propulsor_efficiencies.py
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
def plot_electric_propulsor_efficiencies(results,
                                  save_figure = False,
                                  show_legend = True,
                                  save_filename = "Electric_Efficiencies",
                                  file_type = ".png",
                                  width = 11, height = 7):
    """
    Creates a three-panel plot showing efficiencies of electric propulsion system components.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and propulsion system efficiencies
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag for displaying plot legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Electric_Efficiencies")
        
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
    The function creates a 2x2 subplot containing:
        1. Propulsor efficiency vs time (rotor or ducted fan)
        2. Figure of Merit vs time
        3. Motor efficiency vs time
    
    Each segment is plotted with a different color from the inferno colormap.
    Different propulsors are distinguished by different markers.
    
    **Major Assumptions**
    
    * For identical propulsors, only the first propulsor's data is plotted
    * Time is converted from seconds to minutes for plotting
    * Efficiencies are normalized between 0 and 1
   
    **Definitions**
    
    'Figure of Merit'
        Measure of rotor efficiency comparing actual power to ideal power
    'Motor Efficiency'
        Ratio of mechanical power output to electrical power input
    'Propulsor Efficiency'
        Ratio of useful thrust power to shaft power input
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
    axis_1 = plt.subplot(1,2,1) 
    axis_2 = plt.subplot(1,2,2)


    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for p_i, propulsor in enumerate(network.propulsors):
            if (p_i == 0) or (network.identical_propulsors == False): 
                for i in range(len(results.segments)):  
                    if 'rotor' in propulsor: 
                        thrustor =  propulsor.rotor
                        axis_1.set_ylabel(r'$\eta_{rotor}$')
                    elif 'ducted_fan' in propulsor:
                        thrustor =  propulsor.ducted_fan
                        axis_1.set_ylabel(r'$\eta_{ducted fan}$')
                    motor =  propulsor.motor
                      
                    bus_results  = results.segments[i].conditions.energy 
                    time         = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min      
                    effp         = bus_results[propulsor.tag][thrustor.tag].efficiency[:,0] 
                    effm         = bus_results[propulsor.tag][motor.tag].efficiency[:,0]  
                    
                    if p_i == 0 and i ==0:              
                        axis_1.plot(time, effp, color = line_colors[i], marker = ps.markers[p_i], markersize= ps.marker_size, linewidth = ps.line_width, label = thrustor.tag)
                    else:
                        axis_1.plot(time, effp, color = line_colors[i], marker = ps.markers[p_i], markersize= ps.marker_size, linewidth = ps.line_width) 
                    axis_1.set_ylim([0,1.1])
                    set_axes(axis_1)
                    
                    axis_2.plot(time, effm, color = line_colors[i], marker = ps.markers[p_i], markersize= ps.marker_size, linewidth = ps.line_width)
                    axis_2.set_xlabel('Time (mins)')
                    axis_2.set_ylabel(r'$\eta_{motor}$')
                    axis_2.set_ylim([0,1.1])
                    set_axes(axis_2)
           
    if show_legend:     
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text  =  'Electronic Network Efficiencies' 
    fig.suptitle(title_text)
    if save_figure:
        plt.savefig(save_filename + file_type) 
     
    return fig  