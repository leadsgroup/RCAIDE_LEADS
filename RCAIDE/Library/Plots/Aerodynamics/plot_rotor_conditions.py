## @ingroup Library-Plots-Performance-Aerodynamics
# RCAIDE/Library/Plots/Performance/Aerodynamics/plot_rotor_conditions.py
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
## @ingroup Library-Plots-Performance-Aerodynamics
def plot_rotor_conditions(results,
                        save_figure = False,
                        show_legend=True,
                        save_filename = "Rotor_Conditions",
                        file_type = ".png",
                        width = 11, height = 7):
    """
    Generate plots of key rotor operating conditions over time.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
            - results.segments[0].analyses.energy.vehicle.networks

    save_figure : bool, optional
        Save figure to file if True, default False

    show_legend : bool, optional
        Display segment legend if True, default True

    save_filename : str, optional
        Name for saved figure file, default "Rotor_Conditions"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 7

    Returns
    -------
    fig : matplotlib.figure.Figure

    Notes
    -----
    Creates a 2x2 subplot figure showing:
        - Top left: RPM vs time (minutes)
        - Top right: Rotor tilt angle (degrees) vs time
        - Bottom left: Thrust (N) vs time
        - Bottom right: Torque (N-m) vs time

    Each mission segment uses a different color from the inferno colormap.
    Multiple rotors are distinguished by different markers.

    **Definitions**

    'RPM'
        Rotor rotational speed in revolutions per minute
    
    'Thrust Vector Angle'
        Angle between rotor thrust vector and vertical

    See Also
    --------
    RCAIDE.Library.Plots.Common.set_axes : Standardized axis formatting
    RCAIDE.Library.Plots.Common.plot_style : RCAIDE plot styling
    RCAIDE.Library.Analysis.Performance.propulsion : Analysis modules
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
    axis_1 = plt.subplot(3,2,1)
    axis_2 = plt.subplot(3,2,2)
    axis_3 = plt.subplot(3,2,3) 
    axis_4 = plt.subplot(3,2,4) 
    axis_5 = plt.subplot(3,2,5) 
    axis_6 = plt.subplot(3,2,6)     
 
    for network in results.segments[0].analyses.energy.vehicle.networks: 
        for p_i, propulsor in enumerate(network.propulsors): 
            if (p_i == 0) or (network.identical_propulsors == False):            
                plot_propulsor_data(results,propulsor,axis_1,axis_2,axis_3,axis_4, axis_5, axis_6,line_colors,ps,p_i)                  
              
    if show_legend:                
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend
    fig.tight_layout() 
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text  =  'Rotor Performance' 
    fig.suptitle(title_text)
    if save_figure:
        plt.savefig(save_filename + file_type) 
                 
    return fig 

def plot_propulsor_data(results, propulsor, axis_1, axis_2, axis_3, axis_4, axis_5, axis_6, line_colors, ps, p_i):
    """
    Plot operating conditions data for a single propulsor across mission segments.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
            - results.segments[i].conditions with fields:
                - energy[propulsor_tag][thrustor_tag].rpm
                - energy[propulsor_tag][thrustor_tag].thrust
                - energy[propulsor_tag][thrustor_tag].torque
                - energy[propulsor_tag].commanded_thrust_vector_angle
                - frames.inertial.time

    propulsor : Data
        Propulsor data structure containing:
            - tag : str
                Identifier for the propulsor
            - rotor/propeller : Data
                Thrustor component data

    axis_1 : matplotlib.axes.Axes
        Axis for RPM plot

    axis_2 : matplotlib.axes.Axes
        Axis for rotor angle plot

    axis_3 : matplotlib.axes.Axes
        Axis for thrust plot

    axis_4 : matplotlib.axes.Axes
        Axis for torque plot

    line_colors : array
        Array of RGB colors for different segments

    ps : Data
        Plot style data structure with fields:
            - markers : list
                Marker styles for different propulsors
            - line_width : float
                Width for plot lines

    p_i : int
        Index of current propulsor for marker selection

    Returns
    -------
    None

    Notes
    -----
    Helper function for plot_rotor_conditions that handles plotting
    data for a single propulsor (rotor or propeller) across all mission
    segments.

    The function:
        - Identifies thrustor type (rotor vs propeller)
        - Extracts time histories for each segment
        - Plots RPM, angle, thrust, and torque vs time
        - Applies consistent styling
        - Adds legend for first segment only
        - Converts units as needed (time to minutes, angles to degrees)

    See Also
    --------
    RCAIDE.Library.Plots.Aerodynamics.plot_rotor_conditions : Main plotting function
    """
    if 'rotor' in  propulsor:
        thrustor = propulsor.rotor
    elif 'propeller' in  propulsor:
        thrustor = propulsor.propeller
        
    for i in range(len(results.segments)): 
        bus_results  =  results.segments[i].conditions.energy
        time         =  results.segments[i].conditions.frames.inertial.time[:,0] / Units.min   
        rpm          =  bus_results[propulsor.tag][thrustor.tag].rpm[:,0]
        thrust       =  np.linalg.norm(bus_results[propulsor.tag][thrustor.tag].thrust , axis =1)
        torque       =  bus_results[propulsor.tag][thrustor.tag].torque[:,0]
        eta          =  bus_results[propulsor.tag][thrustor.tag].efficiency[:,0]
        angle        =  bus_results[propulsor.tag].commanded_thrust_vector_angle[:,0]
        beta         =  bus_results[propulsor.tag][thrustor.tag].blade_pitch_command[:,0]
  
        if  i == 0 :
            axis_1.plot(time,rpm, color = line_colors[i], marker = ps.markers[p_i]  , linewidth = ps.line_width, label = thrustor.tag)
        else:
            axis_1.plot(time,rpm, color = line_colors[i], marker = ps.markers[p_i]  , linewidth = ps.line_width)
        axis_1.set_ylabel(r'RPM')
        set_axes(axis_1)    
         
        axis_2.plot(time, angle/Units.degrees, color = line_colors[i], marker = ps.markers[p_i]  , linewidth = ps.line_width) 
        axis_2.set_ylabel(r'Rotor Angle')
        set_axes(axis_2) 
 
        axis_3.plot(time,thrust, color = line_colors[i], marker = ps.markers[p_i] , linewidth = ps.line_width)
        axis_3.set_ylabel(r'Thrust (N)')
        set_axes(axis_3) 
         
        axis_4.plot(time,torque, color = line_colors[i], marker = ps.markers[p_i] , linewidth = ps.line_width)
        axis_4.set_ylabel(r'Torque (N-m)')
        set_axes(axis_4) 

        axis_5.plot(time,beta, color = line_colors[i], marker = ps.markers[p_i] , linewidth = ps.line_width)
        axis_5.set_ylabel(r'Pitch Command')
        set_axes(axis_5)

        axis_6.plot(time,eta, color = line_colors[i], marker = ps.markers[p_i] , linewidth = ps.line_width)
        axis_6.set_ylabel(r'Efficiency')
        set_axes(axis_6)             
    return 