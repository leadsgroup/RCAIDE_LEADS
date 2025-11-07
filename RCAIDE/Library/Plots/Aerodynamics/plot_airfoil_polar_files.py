## @ingroup Library-Plots-Performance-Aerodynamics   
# RCAIDE/Library/Plots/Performance/Aerodynamics/plot_airfoil_polar_files.py
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

## @ingroup Library-Plots-Performance-Aerodynamics   
def plot_airfoil_polar_files(polar_data,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Airfoil_Polars",
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Generate plots of airfoil performance data from polar files.

    Parameters
    ----------
    polar_data : Data
        Airfoil polar data structure containing:
            - lift_coefficients : array
                CL values for each Reynolds number and angle of attack
            - drag_coefficients : array
                CD values for each Reynolds number and angle of attack
            - angle_of_attacks : array
                Angles of attack [rad]
            - reynolds_numbers : array
                Reynolds numbers
            - re_from_polar : array
                List of Reynolds numbers from polar files

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
    Creates a 2x2 subplot figure showing airfoil performance:
        - Top left: Lift coefficient vs angle of attack
        - Top right: Drag coefficient vs angle of attack
        - Bottom left: Drag polar (CL vs CD)
        - Bottom right: Lift-to-drag ratio vs angle of attack

    Each Reynolds number case is plotted in a different color using
    the viridis colormap. Legend entries show Reynolds numbers in
    scientific notation (e.g., "Re=1.0e6").

    See Also
    --------
    RCAIDE.Library.Plots.Common.set_axes : Standardized axis formatting
    RCAIDE.Library.Plots.Common.plot_style : RCAIDE plot styling
    RCAIDE.Library.Analysis.Aerodynamics.process_airfoil_polars : Analysis module
    """ 
  
    
    # get plotting style 
    ps      = plot_style()  
    
    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    
    
    # Get raw data polars
    CL           = polar_data.lift_coefficients
    CD           = polar_data.drag_coefficients
    alpha        = polar_data.angle_of_attacks/Units.degrees
    Re_raw       = polar_data.reynolds_numbers
    n_Re         = len(polar_data.re_from_polar) 
        
     
    # get line colors for plots 
    line_colors   = cm.viridis(np.linspace(0,1,n_Re))     
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
      
    for j in range(n_Re):
        
        Re_val = str(round(Re_raw[j])/1e6)+' x $10^6$'  
        
        axis_1 = plt.subplot(2,2,1)
        axis_1.plot(alpha, CL[j,:], color = line_colors[j], marker = ps.markers[0], linewidth = ps.line_width, label ='Re = '+Re_val)
        axis_1.set_ylabel(r'$C_l$')
        axis_1.set_xlabel(r'$\alpha$')
        set_axes(axis_1)    
        
        axis_2 = plt.subplot(2,2,2)
        axis_2.plot(alpha,CD[j,:], color = line_colors[j], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_2.set_ylabel(r'$C_d$')
        axis_2.set_xlabel(r'$\alpha$')
        set_axes(axis_2)  
        
        axis_3 = plt.subplot(2,2,3)
        axis_3.plot(CD[j,:],CL[j,:], color = line_colors[j], marker = ps.markers[0], linewidth = ps.line_width)
        axis_3.set_xlabel('$C_d$')
        axis_3.set_ylabel(r'$C_l$')
        set_axes(axis_3) 
    
        axis_4 = plt.subplot(2,2,4)
        axis_4.plot(alpha, CL[j,:]/CD[j,:], color = line_colors[j], marker = ps.markers[0], linewidth = ps.line_width) 
        axis_4.set_ylabel(r'$Cl/Cd$')
        axis_4.set_xlabel(r'$\alpha$')
        set_axes(axis_4)   
     

    if show_legend:    
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Airfoil Polars', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8) 
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig