# RCAIDE/Library/Plots/Geometry/plot_airfoil.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
from RCAIDE.Library.Plots.Common import plot_style
from RCAIDE.Library.Methods.Geometry.Airfoil  import import_airfoil_geometry  

# package imports 
import matplotlib.pyplot as plt 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  
def plot_airfoil(airfoil_paths,
                 save_figure = False, 
                 save_filename = "Airfoil_Geometry",
                 file_type = ".png", 
                 width = 11, height = 7):
    """
    Creates a 2D visualization of airfoil geometries from coordinate files.

    Parameters
    ----------
    airfoil_paths : list of str
        Paths to airfoil coordinate files
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Airfoil_Geometry")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Handle to the generated figure

    Notes
    -----
    Creates a plot showing:
        - Airfoil surface coordinates
        - Equal axis scaling
        - Optional figure saving
    
    **Major Assumptions**
    
    * Coordinate files are properly formatted
    * Coordinates are normalized by chord
    * Points are ordered from trailing edge clockwise
    
    **Definitions**
    
    'Chord'
        Line from leading edge to trailing edge
    'Thickness'
        Distance between upper and lower surface
    """
    # get airfoil coordinate geometry     
    airfoil_geometry = import_airfoil_geometry(airfoil_paths)

    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)    

    fig  = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
    axis = fig.add_subplot(1,1,1)     
    axis.plot(airfoil_geometry.x_coordinates,airfoil_geometry.y_coordinates, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width) 
    axis.set_xlabel('x')
    axis.set_ylabel('y')    
     
    if save_figure:
        fig.savefig(save_filename.replace("_", " ") + file_type)  
     
    return fig
