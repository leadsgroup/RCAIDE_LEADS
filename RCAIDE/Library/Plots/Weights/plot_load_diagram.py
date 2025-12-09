# RCAIDE/Library/Plots/Weights/plot_load_diagram.py
# 
# 
# Created:  Aug 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
from shapely.geometry import Polygon
import matplotlib.cm as cm
from scipy.interpolate import griddata
import matplotlib.tri as tri
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ---------------------------------------------------------------------------------------------------------------------- 
def plot_load_diagram(results,
                      save_figure = False,
                      show_legend = True,
                      save_filename = "Aircraft_Loading_Trim_Dragram",
                      file_type = ".png",
                      width = 11, height = 7):
    """
    Creates a comprehensive aircraft loading diagram showing mass and center of gravity relationships.

    Parameters
    ----------
    results : RCAIDE.Framework.Core.Data
        Results from load and trim diagram analysis containing:
            - loading_mass : numpy.ndarray
                Aircraft mass for loading diagram [kg]
            - loading_LEMAC_location : numpy.ndarray
                LEMAC location as percentage of reference chord [%]
            - aerodynamic_LEMAC_location : numpy.ndarray
                LEMAC location for trim diagram [%]
            - aerodynamic_mass : numpy.ndarray
                Aircraft mass for trim diagram [kg]
            - aerodynamic_static_margin : numpy.ndarray
                Static margin values [unitless]
            - MTOW : float
                Maximum takeoff weight [kg]
            - MLW : float
                Maximum landing weight [kg]

    Returns
    -------
    None
        Creates and displays a matplotlib figure with the loading diagram

    Notes
    -----
    This function generates a comprehensive aircraft loading diagram that visualizes
    the relationship between aircraft mass, loading, and center of gravity position. The diagram
    includes fuel loading curves, payload loading curves, weight limits, and stability
    contours to provide a complete view of the aircraft's loading envelope.
    
    **Major Assumptions**
        * Convex hull calculation is valid for the data points
    
    **Definitions**

    'Loading Diagram'
        Plot showing aircraft mass versus center of gravity position for different loading conditions.
    
    'Convex Hull'
        Smallest convex polygon that contains all the data points.
    
    'Static Margin'
        Distance between center of gravity and neutral point as percentage of reference chord.
    
    'LEMAC'
        Leading Edge Mean Aerodynamic Chord reference point for center of gravity calculations.

    See Also
    --------
    matplotlib.pyplot
    scipy.spatial.ConvexHull
    shapely.geometry.Polygon
    """
    
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
 
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)
    axis = fig.add_subplot(1,1,1)
    
    min_range = 0
    max_range = 0
    
    # ------------------------------------------------------------------------    
    # cumulative
    # ------------------------------------------------------------------------ 
    
    # 1. Generate sample scattered data
    points =  np.hstack((   np.atleast_2d(results.loading_LEMAC_location.flatten()).T,  np.atleast_2d(results.loading_mass.flatten()).T )) 
    
    # 2. Compute the convex hull
    hull = ConvexHull(points)
    
    # 3. Extract the vertices of the hull in a counter-clockwise order
    hull_points = points[hull.vertices]
    
    # 4. Create a Shapely Polygon from the hull vertices
    # The vertices are already ordered, but Shapely can handle it
    polygon = Polygon(hull_points) 
    
    # Plot the convex hull polygon boundary
    x_hull, y_hull = polygon.exterior.xy
    
    axis.fill(x_hull, y_hull, color='grey', alpha=0.3, edgecolor='black', linewidth=2)
    axis.plot(x_hull, y_hull, 'k-')

    # ------------------------------------------------------------------------    
    # Maximum Takeoff Weight line
    # ------------------------------------------------------------------------
    x_pts_MTOW = np.linspace(0, 100)
    y_pts_MTOW = np.ones_like(x_pts_MTOW)  * results.MTOW
    axis.plot(x_pts_MTOW, y_pts_MTOW, 'r-', label = 'MTOW') 
    

    # ------------------------------------------------------------------------    
    # Maximum Landing Weight line
    # ------------------------------------------------------------------------
    x_pts_MLW = x_pts_MTOW
    y_pts_MLW = np.ones_like(x_pts_MLW)  * results.MLW
    axis.plot(x_pts_MLW, y_pts_MLW, 'r--', label = 'MLW')  

    # ------------------------------------------------------------------------
    # Stability Contours 
    # ------------------------------------------------------------------------
    SM_levels = np.linspace(-5, 100, 22)
    CS   =  axis.contourf(results.aerodynamic_LEMAC_location, results.aerodynamic_mass, results.aerodynamic_static_margin*100, levels=SM_levels, cmap='viridis') 
    CS2  =  axis.contour(results.aerodynamic_LEMAC_location, results.aerodynamic_mass, results.aerodynamic_static_margin*100, levels=SM_levels, colors='black') 
    cbar = fig.colorbar(CS, ax=axis)
    axis.clabel(CS2, fontsize=10)
    cbar.ax.set_ylabel('Static Margin', rotation =  90)    
    
    # ------------------------------------------------------------------------    
    # Axis Items
    # ------------------------------------------------------------------------     
    axis.set_xlim(0, 100)
    axis.legend(loc='upper right')
    axis.set_xlabel(r'$X_{CG}$ (%MAC)')
    axis.set_ylabel('Mass (kg)')
    axis.grid(True)
    fig.tight_layout()
     
    if save_figure:
        plt.savefig(save_filename + file_type)       
                                  
    return