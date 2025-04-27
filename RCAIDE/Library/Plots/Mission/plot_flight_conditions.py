# RCAIDE/Library/Plots/Performance/Mission/plot_flight_conditions.py
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
def plot_flight_conditions(results,
                           save_figure = False,
                           show_legend = True,
                           save_filename = "Flight Conditions",
                           file_type = ".png",
                           width = 11, height = 7):
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
    axis_1 = fig.add_subplot(2,2,1) 
    axis_2 = fig.add_subplot(2,2,2)
    axis_3 = fig.add_subplot(2,2,3)
    axis_4 = fig.add_subplot(2,2,4)
     
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        airspeed = results.segments[i].conditions.freestream.velocity[:,0] /   Units['mph'] 
        Range    = results.segments[i].conditions.frames.inertial.aircraft_range[:,0]/ Units.nmi
        altitude = results.segments[i].conditions.freestream.altitude[:,0]/Units.feet
        mach     = results.segments[i].conditions.freestream.mach_number[:,0]
              
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')
        
        axis_1.plot(time, altitude, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name)
        
        axis_2.plot(time, airspeed, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
       
        axis_3.plot(time, Range, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
         
        axis_4.plot(time, mach, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)        
        
    axis_1.set_ylabel(r'Altitude (ft)')
    set_axes(axis_1)

    axis_2.set_ylabel(r'Airspeed (mph)')
    set_axes(axis_2) 

    axis_3.set_xlabel('Time (mins)')
    axis_3.set_ylabel(r'Range (nmi)')
    set_axes(axis_3)
    
    axis_4.set_xlabel('Time (mins)')
    axis_4.set_ylabel(r'Mach Number')
    set_axes(axis_4) 
    
    if show_legend:        
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})
        axis_4.legend(loc='upper center')
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.75)
    
    # set title of plot 
    title_text    = 'Flight Conditions'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return  fig 