# RCAIDE/Library/Plots/Mass_Properties/plot_load_diagram.py
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
                      save_figure               = False,
                      show_legend               = True,
                      save_filename             = "Loading_Trim_Dragram",
                      file_type                 = ".png",
                      static_margin_lower_limit = -0.1,
                      static_margin_upper_limit = 1.0,
                      static_margin_resolution  = 12,
                      x_axis_lower_limit        = None,
                      x_axis_upper_limit        = None,
                      y_axis_lower_limit        = None,
                      y_axis_upper_limit        = None,
                      show_component_vectors    = False, 
                      width                     = 11,
                      height                    = 7):
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
            - trim_results.LEMAC_location : numpy.ndarray
                LEMAC location for trim diagram [%]
            - trim_results.mass : numpy.ndarray
                Aircraft mass for trim diagram [kg]
            - trim_results.static_margin : numpy.ndarray
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

    # ------------------------------------------------------------------------
    # Stability Contours 
    # ------------------------------------------------------------------------
    CG_LEMAC       = results.trim_results.CG_percent_of_LEMAC_location*100
    SM             = results.trim_results.static_margin*100
    SM_levels      = np.linspace(static_margin_lower_limit*100, static_margin_upper_limit*100, static_margin_resolution)
    CS             = axis.contourf(CG_LEMAC, results.trim_results.mass, SM, levels = SM_levels, cmap='coolwarm_r', extend='both', alpha = 0.5) 
    CS2            = axis.contour(CG_LEMAC, results.trim_results.mass,SM, levels = SM_levels,  colors='black', extend='both') 
    cbar           = fig.colorbar(CS, ax=axis)
    axis.clabel(CS2, fontsize=10)
    cbar.ax.set_ylabel('Static Margin', rotation =  90)        
    
    # ------------------------------------------------------------------------    
    # load diamonds 
    # ------------------------------------------------------------------------
    # cumulative load vector diamond 
    points =  np.hstack((  100*np.atleast_2d(results.loading_results.CG_percent_of_LEMAC_location.flatten()).T,  np.atleast_2d(results.loading_results.mass.flatten()).T ))  
    hull = ConvexHull(points) 
    hull_points = points[hull.vertices] 
    polygon = Polygon(hull_points)  
    x_hull, y_hull = polygon.exterior.xy
    
    # fuel vector diamond 
    points =  np.hstack((  100*np.atleast_2d(results.loading_results.CG_percent_of_LEMAC_location[:,0,0,:].flatten()).T,  np.atleast_2d(results.loading_results.mass[:,0,0,:].flatten()).T ))  
    hull = ConvexHull(points) 
    hull_points = points[hull.vertices] 
    polygon = Polygon(hull_points)  
    x_hull_f, y_hull_f = polygon.exterior.xy
     
    # passenger vector diamond      
    points =  np.hstack((  100*np.atleast_2d(results.loading_results.CG_percent_of_LEMAC_location[:,:,0,0].flatten()).T,  np.atleast_2d(results.loading_results.mass[:,:,0,0].flatten()).T ))  
    hull = ConvexHull(points) 
    hull_points = points[hull.vertices] 
    polygon = Polygon(hull_points)  
    x_hull_p, y_hull_p = polygon.exterior.xy

    # cargo vector diamond      
    points =  np.hstack((  100*np.atleast_2d(results.loading_results.CG_percent_of_LEMAC_location[:,0,:,0].flatten()).T,  np.atleast_2d(results.loading_results.mass[:,0,:,0].flatten()).T ))  
    hull = ConvexHull(points) 
    hull_points = points[hull.vertices] 
    polygon = Polygon(hull_points)  
    x_hull_c, y_hull_c = polygon.exterior.xy    

    # ------------------------------------------------------------------------    
    # PLot Bounds 
    # ------------------------------------------------------------------------
    x_bound     = max(x_hull) - min(x_hull)
    if x_axis_lower_limit == None: 
        x_axis_lower_limit = min(x_hull) - x_bound / 2
    if x_axis_upper_limit == None: 
        x_axis_upper_limit = max(x_hull) + x_bound / 2
    if y_axis_lower_limit == None: 
        y_axis_lower_limit =  min(y_hull)
    if y_axis_upper_limit == None: 
        y_axis_upper_limit =  max(y_hull) 
    
    # ------------------------------------------------------------------------    
    # Maximum Takeoff Weight line
    # ------------------------------------------------------------------------
    x_pts_MTOW = np.linspace(x_axis_lower_limit, x_axis_upper_limit)
    y_pts_MTOW = np.ones_like(x_pts_MTOW)  * results.MTOW
    axis.plot(x_pts_MTOW, y_pts_MTOW, 'g-', label = 'MTOW') 
    

    # ------------------------------------------------------------------------    
    # Maximum Landing Weight line
    # ------------------------------------------------------------------------
    x_pts_MLW = np.linspace(x_axis_lower_limit, x_axis_upper_limit)
    y_pts_MLW = np.ones_like(x_pts_MLW)  * results.MLW
    axis.plot(x_pts_MLW, y_pts_MLW, 'g-x', label = 'MLW')  
    
    # ------------------------------------------------------------------------    
    # Loading -Trim Bounds  
    # ------------------------------------------------------------------------    
    axis.fill(x_hull, y_hull, color='grey', alpha=0.3, edgecolor='black', linewidth=2)
    axis.plot(x_hull, y_hull, 'k-')
    
    if show_component_vectors:
        axis.fill(x_hull_f, y_hull_f, color = 'goldenrod'     , alpha=0.3      , edgecolor='goldenrod', linewidth=1)
        axis.plot(x_hull_f, y_hull_f, color = 'goldenrod'     , linestyle = '-',label = "Fuel Loading")
        axis.fill(x_hull_p, y_hull_p, color = 'darkred'       , alpha=0.3      , edgecolor='darkred', linewidth=1)
        axis.plot(x_hull_p, y_hull_p, color = 'darkred'       , linestyle = '-',label = "Pax. Loading")
        axis.fill(x_hull_c, y_hull_c, color = 'darksalmon'    , alpha=0.3      , edgecolor='darksalmon', linewidth=1)
        axis.plot(x_hull_c, y_hull_c, color = 'darksalmon'    , linestyle = '-',label = "Cargo Loading")

    # ------------------------------------------------------------------------    
    # Axis Items
    # ------------------------------------------------------------------------     
    axis.set_xlim(x_axis_lower_limit, x_axis_upper_limit) 
    axis.set_ylim(y_axis_lower_limit, y_axis_upper_limit)
    axis.legend(loc='upper right')
    axis.set_xlabel(r'$X_{CG}$/LEMAC (%)')
    axis.set_ylabel('Mass (kg)') 
    plt.grid(False) 
    fig.tight_layout()
     
    if save_figure:
        plt.savefig(save_filename + file_type)       
                                  
    return