# RCAIDE/Library/Plots/Aerodynamics/plot_aircraft_aerodynamic_analysis.py
# 
# 
# Created:  Dec 2024, M. Clarke

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
def plot_aircraft_aerodynamics(results,
                            save_figure = False,
                            line_color = 'bo-',
                            line_color2 = 'rs--',
                            save_filename = "Aircraft_Aerodynamic_Analysis",
                            file_type = ".png",
                            width = 11, height = 7):
    """
    Creates 3D surface plots of aircraft lift and drag coefficients as functions of 
    Mach number and angle of attack.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - Mach : array
                Array of Mach numbers
            - alpha : array
                Array of angles of attack in radians
            - lift_coefficient : array
                2D array of lift coefficients [Mach, alpha]
            - drag_coefficient : array
                2D array of drag coefficients [Mach, alpha]
            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    line_color : str, optional
        Color and style specification for lift plot (default: 'bo-')
        
    line_color2 : str, optional
        Color and style specification for drag plot (default: 'rs--')
        
    save_filename : str, optional
        Name of file for saved figure (default: "Aircraft_Aerodynamic_Analysis")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    None


    Notes
    -----
    Creates visualization showing:
        - Aerodynamic coefficient variations
        - Mach number effects
        - Angle of attack sensitivity
        - Nonlinear aerodynamic behavior
        
    Function displays and optionally saves two 3D surface plots:
        Left Panel:
            - Lift coefficient surface
            - Shows CL variation with Mach and alpha
            - X-axis: Mach number
            - Y-axis: Angle of attack [degrees]
            - Z-axis: Lift coefficient
            
        Right Panel:
            - Drag coefficient surface
            - Shows CD variation with Mach and alpha
            - X-axis: Mach number
            - Y-axis: Angle of attack [degrees]
            - Z-axis: Drag coefficient
    
    **Definitions**
    
    'Mach Number'
        Ratio of airspeed to speed of sound
    'Angle of Attack'
        Angle between airflow and reference line
    'Lift Coefficient'
        Non-dimensional lift force
    'Drag Coefficient'
        Non-dimensional drag force
    
    See Also
    --------
    plot_aerodynamic_coefficients : Time history of coefficients
    plot_drag_components : Drag breakdown analysis
    """
    

    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    
    #------------------------------------------------------------------------
    # setup figures
    #------------------------------------------------------------------------
    fig = plt.figure(save_filename + " Lift Coefficients")  
    fig.set_size_inches(12,6) 
    axis_1 = fig.add_subplot(1, 2, 1, projection='3d')
    axis_2 = fig.add_subplot(1, 2, 2, projection='3d') 
 
    X, Y = np.meshgrid(results.Mach, results.alpha)
    surf = axis_1.plot_surface(X, Y/Units.degree, results.lift_coefficient   , cmap=cm.jet,linewidth=0, antialiased=False) 
    surf = axis_2.plot_surface(X, Y/Units.degree, results.drag_coefficient   , cmap=cm.jet,linewidth=0, antialiased=False) 

    axis_1.set_title('$C_L$') 
    axis_2.set_title('$C_D$')            
    axis_1.set_ylabel('AoA') 
    axis_2.set_ylabel('AoA')  
    axis_1.set_xlabel('Mach') 
    axis_2.set_xlabel('Mach')   
    
    # set title of plot 
    title_text    = 'Aircraft Aerodynamic Analysis '    
    fig.suptitle(title_text) 
    
    plt.tight_layout()    
    if save_figure:    
        fig.savefig(save_filename + file_type) 
    
    plt.tight_layout()
    return
