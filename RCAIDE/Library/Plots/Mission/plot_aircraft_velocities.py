# RCAIDE/Library/Plots/Performance/Mission/plot_aircraft_velocities.py
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
def plot_aircraft_velocities(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Aircraft Velocities",
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates a multi-panel visualization of aircraft velocity components over a mission.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing segment conditions
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Aircraft Velocities")
        
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
            - True airspeed
            - Equivalent airspeed
            - Calibrated airspeed
            - Mach number

    Notes
    -----
    Creates a four-panel plot showing:
        1. True airspeed vs time
        2. Equivalent airspeed vs time
        3. Calibrated airspeed vs time
        4. Mach number vs time
    
    **Major Assumptions**
    
    * Results contain freestream conditions
    * Time is in minutes
    * Velocities are in knots
    * Standard atmosphere density used as reference

    **Definitions**
    
    'True Airspeed'
        Actual aircraft velocity relative to airmass
    'Equivalent Airspeed'
        Velocity at sea level producing same dynamic pressure
    'Calibrated Airspeed'
        Indicated airspeed corrected for instrument and position errors
    'Mach Number'
        Ratio of true airspeed to local speed of sound
    
    See Also
    --------
    RCAIDE.Library.Plots.Mission.plot_flight_conditions : Related atmospheric conditions
    RCAIDE.Library.Plots.Mission.plot_flight_trajectory : Complete trajectory visualization
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
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        velocity = results.segments[i].conditions.freestream.velocity[:,0] / Units.kts
        density  = results.segments[i].conditions.freestream.density[:,0]
        PR       = density/1.225
        EAS      = velocity * np.sqrt(PR)
        mach     = results.segments[i].conditions.freestream.mach_number[:,0]
        CAS      = EAS * (1+((1/8)*((1-PR)*mach**2))+((3/640)*(1-10*PR+(9*PR**2)*(mach**4))))

             
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')
        axis_1 = plt.subplot(2,2,1)
        axis_1.plot(time, velocity, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name)
        axis_1.set_ylabel(r'True Airspeed (kts)')
        set_axes(axis_1)    
        
        axis_2 = plt.subplot(2,2,2)
        axis_2.plot(time, EAS, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_2.set_ylabel(r'Equiv. Airspeed (kts)')
        set_axes(axis_2) 

        axis_3 = plt.subplot(2,2,3)
        axis_3.plot(time, CAS, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_3.set_xlabel('Time (mins)')
        axis_3.set_ylabel(r'Calibrated Airspeed (kts)')
        set_axes(axis_3) 
        
        axis_4 = plt.subplot(2,2,4)
        axis_4.plot(time, mach, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)
        axis_4.set_xlabel('Time (mins)')
        axis_4.set_ylabel(r'Mach Number')
        set_axes(axis_4) 
    
    
    if show_legend:    
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend
    fig.tight_layout() 
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Aircraft Speeds'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 