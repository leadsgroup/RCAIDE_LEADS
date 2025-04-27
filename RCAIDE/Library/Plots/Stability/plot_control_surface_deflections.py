# RCAIDE/Library/Plots/Performance/Mission/plot_control_surface_deflections.py
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
def plot_control_surface_deflections(results,
                           save_figure = False,
                           show_legend = True,
                           save_filename = "Flight Conditions",
                           file_type = ".png",
                           width = 6, height = 4):
    """
    Creates a multi-panel visualization of flight conditions over a mission profile.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
        
        - segments[i].conditions.frames.inertial.time[:,0]
            Time history for each segment
        - segments[i].conditions.freestream.velocity[:,0]
            Airspeed history for each segment
        - segments[i].conditions.frames.body.inertial_rotations[:,1,None]
            Pitch angle history for each segment
        - segments[i].conditions.frames.inertial.aircraft_range[:,0]
            Range history for each segment
        - segments[i].conditions.freestream.altitude[:,0]
            Altitude history for each segment
        - segments[i].tag
            Name/identifier of each segment
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Flight Conditions")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Handle to the generated figure containing four subplots:
            - Altitude vs time
            - Airspeed vs time
            - Range vs time
            - Pitch angle vs time

    Notes
    -----
    Creates a four-panel plot showing:
        1. Altitude profile
        2. Airspeed variation
        3. Range covered
        4. Aircraft pitch attitude
    
    Requires the following data in results:
        - frames.inertial.time
        - frames.inertial.position_vector
        - frames.body.inertial_rotations
        - freestream.velocity
        - freestream.altitude
    
    **Major Assumptions**
    
    * Time is in minutes
    * Altitude is in feet
    * Airspeed is in mph
    * Range is in nautical miles
    * Pitch angle is in degrees
    
    **Definitions**
    
    'Altitude'
        Height above reference plane
    'Range'
        Ground distance covered
    'Pitch Angle'
        Nose-up/down attitude relative to horizon
    
    See Also
    --------
    RCAIDE.Library.Plots.Mission.plot_aircraft_velocities : Detailed velocity analysis
    RCAIDE.Library.Plots.Mission.plot_flight_trajectory : 3D trajectory visualization
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
    axis = fig.add_subplot(2,2,4)
    
 
    aileron_flag   = False
    elevator_flag  = False 
    rudder_flag    = False 
    slat_flag      = False
    flap_flag      = False
    spoiler_flag   = False 

    # loop through wings to determine what control surfaces are present  
    for wing in results.segments[0].analyses.aerodynamics.vehicle.wings: 
        for control_surface in wing.control_surfaces:  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                aileron_flag = True
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator:
                elevator_flag =  True
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:
                rudder_flag =  True
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:
                slat_flag =  True
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                flap_flag = True
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler:
                spoiler_flag = True
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min 
        
        if i == 0:
            if elevator_flag:
                elevator_deflection =  results.segments[i].conditions.control_surfaces.elevator.deflection[:,0] / Units.deg
                axis.plot(time, elevator_deflection, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = ' elevator ' )
            if flap_flag:
                flap_deflection     =  results.segments[i].conditions.control_surfaces.flap.deflection[:,0] / Units.deg
                axis.plot(time, flap_deflection    , color = line_colors[i], marker = ps.markers[2], linewidth = ps.line_width, label = ' flap' )
            if slat_flag:
                slat_deflection     =  results.segments[i].conditions.control_surfaces.slat.deflection[:,0] / Units.deg
                axis.plot(time, slat_deflection    , color = line_colors[i], marker = ps.markers[3], linewidth = ps.line_width, label = ' slat' )
            if aileron_flag:
                aileron_deflection  =  results.segments[i].conditions.control_surfaces.aileron.deflection[:,0] / Units.deg
                axis.plot(time, aileron_deflection , color = line_colors[i], marker = ps.markers[4], linewidth = ps.line_width, label = ' aileron' )
            if rudder_flag: 
                rudder_deflection   =  results.segments[i].conditions.control_surfaces.rudder.deflection[:,0] / Units.deg
                axis.plot(time, rudder_deflection  , color = line_colors[i], marker = ps.markers[5], linewidth = ps.line_width, label = ' rudder' ) 
            if spoiler_flag: 
                spoiler_deflection   =  results.segments[i].conditions.control_surfaces.spoiler.deflection[:,0] / Units.deg  
                axis.plot(time, spoiler_deflection  , color = line_colors[i], marker = ps.markers[6], linewidth = ps.line_width, label = ' spoiler' ) 
        else:
            if elevator_flag:
                elevator_deflection =  results.segments[i].conditions.control_surfaces.elevator.deflection[:,0] / Units.deg
                axis.plot(time, elevator_deflection, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
            if flap_flag:
                flap_deflection     =  results.segments[i].conditions.control_surfaces.flap.deflection[:,0] / Units.deg                
                axis.plot(time, flap_deflection    , color = line_colors[i], marker = ps.markers[2], linewidth = ps.line_width)
            if slat_flag:
                slat_deflection     =  results.segments[i].conditions.control_surfaces.slat.deflection[:,0] / Units.deg 
                axis.plot(time, slat_deflection    , color = line_colors[i], marker = ps.markers[3], linewidth = ps.line_width)
            if aileron_flag:
                aileron_deflection  =  results.segments[i].conditions.control_surfaces.aileron.deflection[:,0] / Units.deg
                axis.plot(time, aileron_deflection , color = line_colors[i], marker = ps.markers[4], linewidth = ps.line_width)
            if rudder_flag: 
                rudder_deflection   =  results.segments[i].conditions.control_surfaces.rudder.deflection[:,0] / Units.deg
                axis.plot(time, rudder_deflection  , color = line_colors[i], marker = ps.markers[5], linewidth = ps.line_width)
            if spoiler_flag: 
                spoiler_deflection   =  results.segments[i].conditions.control_surfaces.spoiler.deflection[:,0] / Units.deg  
                axis.plot(time, spoiler_deflection  , color = line_colors[i], marker = ps.markers[6], linewidth = ps.line_width)             
         
    axis.set_xlabel('Time (mins)')
    axis.set_ylabel(r'Ctrl Surf. Defl. (deg)')
    set_axes(axis)
    
    if show_legend:        
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})
        axis.legend(loc='upper center')
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.75)
    
    # set title of plot 
    title_text    = 'Flight Conditions'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return  fig 