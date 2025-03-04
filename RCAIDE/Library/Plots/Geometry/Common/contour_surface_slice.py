## @ingroup Library-Plots-Geometry-Common
# RCAIDE/Library/Plots/Geometry/Common/contour_surface_slices.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import plotly.graph_objects as go   

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  -Common
def contour_surface_slice(x, y, z, values, color_scale, showscale = False, 
                         colorbar_title = None, colorbar_location = 'right', 
                         colorbar_orientation = 'v'):
    """
    Creates a 3D surface plot with contour coloring for visualization of geometry slices.

    Parameters
    ----------
    x : ndarray
        2D array of x-coordinates defining surface points
        
    y : ndarray
        2D array of y-coordinates defining surface points
        
    z : ndarray
        2D array of z-coordinates defining surface points
        
    values : ndarray
        2D array of values used for surface coloring
        
    color_scale : str
        Color scale specification for surface visualization
        
    showscale : bool, optional
        Flag to display the color scale bar (default: False)
        
    colorbar_title : str, optional
        Title for the color scale bar (default: None)
        
    colorbar_location : str, optional
        Location of color scale bar ('right', 'left', 'top', 'bottom') (default: 'right')
        
    colorbar_orientation : str, optional
        Orientation of color scale bar ('v' for vertical, 'h' for horizontal) (default: 'v')

    Returns
    -------
    surface : plotly.graph_objects.Surface
        Surface object ready for plotting

    Notes
    -----
    Creates a surface visualization with:

        - Continuous color mapping based on values
        - Optional color scale bar
        - Customizable appearance
    
    **Major Assumptions**
    
    * Input arrays are 2D and of matching dimensions
    * Values array corresponds to surface points
    * Color scale is valid for plotly
    """
    return go.Surface(
        x=x, 
        y=y, 
        z=z, 
        surfacecolor=values, 
        colorscale=color_scale, 
        showscale=showscale, 
        colorbar=dict(
            title=dict(text=colorbar_title),  # Use a dictionary for title
            orientation="v"  # Vertical orientation (default)
        )
    )
   
