# RCAIDE/Library/Plots/Aerodynamics/plot_rotor_performance.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import pandas as pd 
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 
# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------      
def plot_rotor_performance(rotor,
                           outputs,
                           title       = None,
                           show_figure = True, 
                           save_figure = False,
                           show_legend = True, 
                           line_colors = ['black', 'blue'], 
                           save_filename='Rotor_Performance',
                           file_type = ".png",
                           width = 11, height = 7):
    """
    Generate plots summarizing rotor aerodynamic performance distributions.

    Parameters
    ----------
    rotor : Data
        Rotor data structure containing outputs with fields:

        - disc_radial_distribution : array
            Radial positions on disc [m]
        - disc_axial_velocity : array
            Total axial velocity [m/s]
        - disc_tangential_velocity : array
            Total tangential velocity [m/s]
        - disc_axial_induced_velocity : array
            Induced axial velocity [m/s]
        - disc_tangential_induced_velocity : array
            Induced tangential velocity [m/s]
        - disc_thrust_distribution : array
            Local thrust distribution [N]
        - disc_torque_distribution : array
            Local torque distribution [N-m]

    title : str, optional
        Custom plot title, default None

    show_figure : bool, optional
        Display figure if True, default True

    save_figure : bool, optional
        Save figure to file if True, default False

    save_filename : str, optional
        Name for saved figure file, default 'Rotor_Performance'

    file_type : str, optional
        File extension for saved figure, default ".png"

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure containing four subplots:

        - Velocity distributions
        - Induced velocity distributions
        - Thrust distribution
        - Torque distribution

    Notes
    -----
    Each subplot includes:

    - Appropriate axis labels
    - Legend identifying components
    - Consistent line styling

    **Definitions**

    'Total Velocity'
        Sum of freestream and induced velocities
    
    'Induced Velocity'
        Additional velocity induced by rotor
    
    'Thrust Distribution'
        Local thrust force per unit radius
    
    'Torque Distribution'
        Local torque per unit radius

    See Also
    --------
    plot_rotor_disc_performance : Detailed disc visualization
    """
    # unpack 
    r_distribution = outputs.disc_radial_distribution[0, :, 0]
     
    
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
    axis_2 = plt.subplot(2,2,2) 
    axis_3 = plt.subplot(2,2,3) 
    axis_4 = plt.subplot(2,2,4)  
    
    axis_1.plot(r_distribution, outputs.disc_axial_velocity[0, :, 0]     , color = line_colors[0],  label = 'Axial')
    axis_1.plot(r_distribution, outputs.disc_tangential_velocity[0, :, 0], color = line_colors[1],  label = 'Tangential')
    axis_2.plot(r_distribution, outputs.disc_axial_induced_velocity[0, :, 0], color = line_colors[0],  label = 'Axial') 
    axis_2.plot(r_distribution, outputs.disc_tangential_induced_velocity[0, :, 0], color = line_colors[1],  label = 'Tangential')  
    axis_3.plot(r_distribution, outputs.disc_thrust_distribution[0, :, 0], color = line_colors[0],  label = 'Thrust') 
    axis_4.plot(r_distribution, outputs.disc_torque_distribution[0, :, 0], color = line_colors[0],  label = 'Torque') 

    axis_1.set_xlabel(r'Radial Station')     
    axis_1.set_ylabel(r'Velocity')  
    axis_2.set_xlabel(r'Radial Station')     
    axis_2.set_ylabel(r'Induced Velocity')  
    axis_3.set_xlabel(r'Radial Station')     
    axis_3.set_ylabel(r'Thrust, N')  
    axis_4.set_xlabel(r'Radial Station')     
    axis_4.set_ylabel(r'Torque, N-m') 
    
    set_axes(axis_1)  
    set_axes(axis_2)  
    set_axes(axis_3)  
    set_axes(axis_4)  
      
    if show_legend:    
        axis_1.legend()  
        axis_2.legend()  
        axis_3.legend()  
        axis_4.legend()   
    
    # set title of plot 
    title_text    = 'Rotor Performance'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)  
    return fig 
 