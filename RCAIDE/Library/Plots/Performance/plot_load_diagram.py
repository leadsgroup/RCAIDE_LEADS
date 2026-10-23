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
from scipy.spatial import ConvexHull, QhullError
from shapely.geometry import Polygon, LineString, box
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
                      save_filename             = "Aircraft_Loading_Trim_Dragram",
                      file_type                 = ".png",
                      static_margin_lower_limit = -0.1,
                      static_margin_upper_limit = 1.0,
                      static_margin_resolution  = 12,
                      x_axis_lower_limit        = None,
                      x_axis_upper_limit        = None,
                      y_axis_lower_limit        = None,
                      y_axis_upper_limit        = None,
                      show_component_vectors    = False,
                      color_map                 = 'coolwarm_r',
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

    # neutral point in %MAC; static margin depends only on CG position, SM = NP - CG
    SM_levels      = np.linspace(static_margin_lower_limit*100, static_margin_upper_limit*100, static_margin_resolution)
    NP_percent_MAC = np.mean(results.trim_results.static_margin + results.trim_results.CG_percent_of_LEMAC_location)*100

    # ------------------------------------------------------------------------    
    # load diamonds 
    # ------------------------------------------------------------------------
    # cumulative, fuel, passenger and cargo load vector diamonds, clipped at MTOW
    CG_LEMAC_load = 100*results.loading_results.CG_percent_of_LEMAC_location
    mass_load     = results.loading_results.mass
    x_hull  , y_hull   = compute_loading_hull(CG_LEMAC_load, mass_load, results.MTOW)
    x_hull_f, y_hull_f = compute_loading_hull(CG_LEMAC_load[:,0,0,:], mass_load[:,0,0,:], results.MTOW)
    x_hull_p, y_hull_p = compute_loading_hull(CG_LEMAC_load[:,:,0,0], mass_load[:,:,0,0], results.MTOW)
    x_hull_c, y_hull_c = compute_loading_hull(CG_LEMAC_load[:,0,:,0], mass_load[:,0,:,0], results.MTOW)

    # ------------------------------------------------------------------------    
    # PLot Bounds 
    # ------------------------------------------------------------------------
    x_bound     = max(x_hull) - min(x_hull)
    if x_axis_lower_limit == None: 
        x_axis_lower_limit = min(x_hull) - 0.1 * x_bound
    if x_axis_upper_limit == None: 
        x_axis_upper_limit = max(x_hull) + 0.1 * x_bound
    y_bound     = results.MTOW - min(y_hull)
    if y_axis_lower_limit == None:
        y_axis_lower_limit =  min(y_hull) - 0.05 * y_bound
    if y_axis_upper_limit == None:
        y_axis_upper_limit =  results.MTOW + 0.05 * y_bound

    # ------------------------------------------------------------------------
    # Stability Contours over the full plot area
    # ------------------------------------------------------------------------
    CG_grid, mass_grid = np.meshgrid(np.linspace(x_axis_lower_limit, x_axis_upper_limit, 200),
                                     np.linspace(y_axis_lower_limit, y_axis_upper_limit, 2))
    SM             = NP_percent_MAC - CG_grid
    CS             = axis.contourf(CG_grid, mass_grid, SM, levels = SM_levels, cmap=color_map, extend='both', alpha = 0.5)
    CS2            = axis.contour(CG_grid, mass_grid, SM, levels = SM_levels,  colors='black', extend='both')
    cbar           = fig.colorbar(CS, ax=axis, format='%.0f')
    y_label        = 0.5 * (y_axis_lower_limit + y_axis_upper_limit)
    label_points   = [(NP_percent_MAC - level, y_label) for level in CS2.levels[::2] if x_axis_lower_limit < NP_percent_MAC - level < x_axis_upper_limit]
    axis.clabel(CS2, fontsize=10, fmt='%.0f%%', manual=label_points, inline=True)
    cbar.ax.set_ylabel('Static Margin (%)', rotation =  90)

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
    axis.legend(loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=3, frameon=False)
    axis.set_xlabel(r'$X_{CG}$/LEMAC (%)')
    axis.set_ylabel('Mass (kg)') 
    plt.grid(False) 
    fig.tight_layout()
     
    if save_figure:
        plt.savefig(save_filename + file_type)       
                                  
    return

def compute_loading_hull(CG_LEMAC, mass, max_mass):
    """Outline of a set of loading points clipped at max_mass; a line through them when they span no area (e.g. no cargo bays)."""
    points = np.column_stack((CG_LEMAC.flatten(), mass.flatten()))
    limit  = box(points[:,0].min() - 1, points[:,1].min() - 1, points[:,0].max() + 1, max_mass)
    try:
        hull = ConvexHull(points)
    except QhullError:
        order   = np.lexsort((points[:,0], points[:,1]))
        clipped = LineString(points[order]).intersection(limit) if len(np.unique(points, axis=0)) > 1 else None
        if clipped is None or clipped.is_empty or clipped.geom_type != 'LineString':
            keep = points[order][points[order][:,1] <= max_mass]
            return keep[:,0], keep[:,1]
        x_line, y_line = clipped.xy
        return np.array(x_line), np.array(y_line)
    x_hull, y_hull = Polygon(points[hull.vertices]).intersection(limit).exterior.xy
    return np.array(x_hull), np.array(y_hull)
