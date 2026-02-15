# RCAIDE/Library/Plots/Performance/Mission/plot_flight_trajectory.py
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
def plot_flight_trajectory(results,
                           line_color = 'bo-',
                           line_color2 = 'rs--',
                           save_figure = False,
                           show_legend = True,
                           save_filename = "Flight_Trajectory",
                           file_type = ".png",
                           width = 11, height = 7):
    """
    Creates a multi-panel visualization of aircraft trajectory and mission profile.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
        
        - segments[i].conditions.frames.inertial.time[:,0]
            Time history for each segment
        - segments[i].conditions.frames.inertial.aircraft_range[:,0]
            Range history for each segment
        - segments[i].conditions.frames.inertial.position_vector[:,0:3]
            3D position vector containing:
                - [:,0]: x-position
                - [:,1]: y-position
                - [:,2]: z-position
        - segments[i].tag
            Name/identifier of each segment
            
    line_color : str, optional
        Primary line style specification (default: 'bo-')
        
    line_color2 : str, optional
        Secondary line style specification (default: 'rs--')
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag to display segment legend (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Flight_Trajectory")
        
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
            - Range vs time
            - Top view (x-y plane)
            - Altitude vs time
            - 3D trajectory

    Notes
    -----
    Creates a four-panel plot showing:
        1. Mission range profile
        2. Top-down view of flight path
        3. Altitude profile
        4. Complete 3D trajectory
    
    **Major Assumptions**
    
    * Time is in minutes
    * Range is in nautical miles
    * Position coordinates are in meters
    * Z-axis points downward in inertial frame
    
    **Definitions**
    
    'Range'
        Total ground distance covered
    'Ground Track'
        Projection of flight path onto x-y plane
    'Altitude Profile'
        Variation of height with time
    
    See Also
    --------
    RCAIDE.Library.Plots.Mission.plot_flight_conditions : Detailed flight condition analysis
    RCAIDE.Library.Plots.Mission.plot_aircraft_velocities : Aircraft speed profiles
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
     
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        Range    = results.segments[i].conditions.frames.inertial.aircraft_range[:,0]/Units.nmi
        x        = results.segments[i].conditions.frames.inertial.position_vector[:,0]  
        y        = results.segments[i].conditions.frames.inertial.position_vector[:,1] 
        z        = -results.segments[i].conditions.frames.inertial.position_vector[:,2] 

        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')
        
        axes = plt.subplot(2,2,1)
        axes.plot( time , Range, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width , label = segment_name)
        axes.set_ylabel('Distance (nmi)')
        axes.set_xlabel('Time (min)')
        set_axes(axes)            

        axes = plt.subplot(2,2,2)
        axes.plot(x, y , line_color, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width )
        axes.set_xlabel('x (m)')
        axes.set_ylabel('y (m)')
        set_axes(axes)

        axes = plt.subplot(2,2,3)
        axes.plot( time , z, line_color , color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width )
        axes.set_ylabel('z (m)')
        axes.set_xlabel('Time (min)')
        set_axes(axes)   
        
        axes = plt.subplot(2,2,4, projection='3d') 
        axes.scatter(x, y, z, marker='o',color =  line_colors[i])
        axes.set_xlabel('x')
        axes.set_ylabel('y')
        axes.set_zlabel('z')
        axes.set_box_aspect([1,1,1])
        set_axes(axes)         
        
    if show_legend:        
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Flight Trajectory'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
             
    return fig         