# RCAIDE/Library/Plots/Performance/Aerodynamics/plot_airfoil_polars.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------     

## @ingroup Library-Plots-Performance-Aerodynamics   
def plot_airfoil_polars(polar_data,
                        save_figure = False,
                        save_filename = "Airfoil_Polars",
                        file_type = ".png",
                        width = 11, height = 7):
    """
    Generate plots of airfoil aerodynamic characteristics from analysis results.

    Parameters
    ----------
    polar_data : Data
        Airfoil analysis results containing:
            - cl_invisc[0] : array
                Inviscid lift coefficients
            - cd_invisc[0] : array
                Inviscid drag coefficients
            - cm_invisc[0] : array
                Inviscid moment coefficients
            - AoA[0] : array
                Angles of attack [rad]
            - Re[0] : array
                Reynolds numbers

    save_figure : bool, optional
        Save figure to file if True, default False

    save_filename : str, optional
        Name for saved figure file, default "Airfoil_Polars"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 7

    Returns
    -------
    fig : matplotlib.figure.Figure

    Notes
    -----
    Creates a 2x2 subplot figure showing inviscid airfoil characteristics:
        - Top left: Lift coefficient vs angle of attack
        - Top right: Drag coefficient vs angle of attack
        - Bottom left: Moment coefficient vs angle of attack
        - Bottom right: Lift-to-drag ratio vs angle of attack

    Results are plotted for a single Reynolds number case, with the
    Reynolds number shown in the legend (e.g., "Re=1.0e6").

    See Also
    --------
    RCAIDE.Library.Plots.Common.set_axes : Standardized axis formatting
    RCAIDE.Library.Plots.Common.plot_style : RCAIDE plot styling
    RCAIDE.Library.Analysis.Aerodynamics.compute_airfoil_inviscid : Analysis module
    RCAIDE.Library.Plots.Aerodynamics.plot_airfoil_polar_files : Viscous polar plotting function
    """ 
 
    # Get raw data polars 
    CL           = polar_data.cl_invisc[0]
    CD           = polar_data.cd_invisc[0]
    CM           = polar_data.cm_invisc[0]
    alpha        = polar_data.AoA[0]/Units.degrees
    Re_raw       = polar_data.Re[0]  
       
    Re_val = str(round(Re_raw[0])/1e6)+'e6' 
    
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
      
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
               
    axis_1 = plt.subplot(2,2,1)
    axis_1.plot(alpha, CL, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width, label = 'Re = '+Re_val)
    axis_1.set_ylabel(r'$C_l$')
    set_axes(axis_1)    
    
    axis_2 = plt.subplot(2,2,2)
    axis_2.plot(alpha, CD, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width)
    axis_2.set_ylabel(r'$C_d$')
    set_axes(axis_2) 

    axis_3 = plt.subplot(2,2,3)
    axis_3.plot(alpha, CM, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width)
    axis_3.set_xlabel('AoA [deg]') 
    axis_3.set_ylabel(r'$C_m$')
    set_axes(axis_3) 
    
    axis_4 = plt.subplot(2,2,4)
    axis_4.plot(alpha, CL/CD, color = ps.color, marker = ps.markers[0], linewidth = ps.line_width)
    axis_4.set_xlabel('AoA [deg]')
    axis_4.set_ylabel(r'Cl/Cd')
    axis_4.set_ylim([-20,20])
    set_axes(axis_4) 
            
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return  fig
     
     
     
     