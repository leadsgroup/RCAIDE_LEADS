## @ingroup Library-Plots-Performance-Aerodynamics  
# RCAIDE/Library/Plots/Performance/Aerodynamics/plot_airfoil_boundary_layer_properties.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style

# python imports 
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------     

def plot_airfoil_boundary_layer_properties(ap,
                                           save_figure = False,
                                           show_legend = False,
                                           file_type = ".png",
                                           save_filename = 'Airfoil_with_Boundary_Layers', 
                                           width = 11, height = 7):
    """
    Generate plots of airfoil boundary layer properties and distributions.

    Parameters
    ----------
    ap : Data
        Airfoil properties data structure containing:
            - x, y : arrays
                Airfoil surface coordinates
            - y_bl : array
                Boundary layer displacement thickness coordinates
            - Ue_Vinf : array
                Edge velocity ratio (Ue/U∞)
            - H : array
                Shape factor
            - delta_star : array
                Displacement thickness
            - delta : array
                Boundary layer thickness
            - theta : array
                Momentum thickness
            - cf : array
                Skin friction coefficient
            - Re_theta : array
                Momentum thickness Reynolds number
            - AoA : array
                Angles of attack [rad]
            - Re : array
                Reynolds numbers

    save_figure : bool, optional
        Save figures to files if True, default False

    show_legend : bool, optional
        Display case legend if True, default False

    file_type : str, optional
        File extension for saved figures, default ".png"

    save_filename : str, optional
        Base name for saved figure files, default 'Airfoil_with_Boundary_Layers'

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 7

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing airfoil with boundary layers

    Notes
    -----
    Creates multiple figures showing:
        - Airfoil profile with boundary layer thickness
        - Edge velocity distribution
        - Shape factor distribution
        - Displacement thickness distribution
        - Boundary layer thickness distribution
        - Momentum thickness distribution
        - Skin friction distribution
        - Momentum thickness Reynolds number distribution

    Each case (combination of AoA and Re) is plotted in a different color.
    Upper and lower surfaces are distinguished by line style.

    **Definitions**

    'Shape Factor'
        Ratio of displacement to momentum thickness
    
    'Displacement Thickness'
        Distance flow is displaced from surface
    
    'Momentum Thickness'
        Loss of momentum flux in boundary layer

    See Also
    --------
    RCAIDE.Library.Plots.Common.set_axes : Standardized axis formatting
    RCAIDE.Library.Plots.Common.plot_style : RCAIDE plot styling
    RCAIDE.Library.Analysis.Aerodynamics.process_airfoil_boundary_layer : Analysis module
    """      
    # get plotting style 
    ps      = plot_style()  
    
    plot_quantity(ap, ap.Ue_Vinf, r'U_e/U_inf$'  ,'Inviscid Edge Velocity',0,3,file_type,show_legend,save_figure,width,height) 
    plot_quantity(ap, ap.H,  r'H'  ,'Kinematic Shape Parameter',-1,10,file_type,show_legend,save_figure,width,height) 
    plot_quantity(ap, ap.delta_star, r'delta*' ,'Displacement Thickness',-0.01,0.1 ,file_type,show_legend,save_figure,width,height) 
    plot_quantity(ap, ap.delta   , r'delta' ,'Boundary Layer Thickness',-0.01,0.1 ,file_type,show_legend,save_figure,width,height) 
    plot_quantity(ap, ap.theta, r'theta' ,'Momentum Thickness',-0.001, 0.015,file_type,show_legend,save_figure,width,height) 
    plot_quantity(ap, ap.cf, r'c_f'  ,   'Skin Friction Coefficient',-0.1,1,file_type,show_legend,save_figure,width,height) 
    plot_quantity(ap, ap.Re_theta,  r'Re_theta'  ,'Theta Reynolds Number',-2E2,1E3,file_type,show_legend,save_figure,width,height)  
 
    n_cpts   = len(ap.AoA[:,0])
    n_cases  = len(ap.AoA[0,:]) 

    # create array of colors for difference reynolds numbers        
    blues = cm.winter(np.linspace(0,0.9,n_cases))     
    reds  = cm.autumn(np.linspace(0,0.9,n_cases))   

    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)
    
    for i in range(n_cpts):   
        for j in range(n_cases):  
            axis_0 = plt.subplot(1,1,1) 
            axis_0.plot(ap.x[i,j], ap.y[i,j], color = blues[j], marker = ps.markers[0], linewidth = ps.line_width )
            axis_0.plot(ap.x[i,j][:-1], ap.y_bl[i,j], color = reds[j], marker = ps.markers[0], linewidth = ps.line_width ) 
            set_axes(axis_0)    
   
    # set title of plot 
    title_text    = 'Airfoil with Boundary Layers'  
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
 
    return fig    
 
# ----------------------------------------------------------------------
#  Plot Quantity
# ----------------------------------------------------------------------  
def plot_quantity(ap, q, qaxis, qname, ylim_low, ylim_high, file_type, show_legend, save_figure, width, height):
    """
    Generate standardized plots of boundary layer quantities along airfoil surfaces.

    Parameters
    ----------
    ap : Data
        Airfoil properties data structure containing:
            - x : array
                Airfoil surface coordinates
            - AoA : array
                Angles of attack [rad]
            - Re : array
                Reynolds numbers

    q : array
        Values of quantity to plot along airfoil surface
    
    qaxis : str
        LaTeX-formatted axis label for quantity
    
    qname : str
        Descriptive name for quantity (used in title)
    
    ylim_low : float
        Lower limit for y-axis
    
    ylim_high : float
        Upper limit for y-axis
    
    file_type : str
        File extension for saved figure
    
    show_legend : bool
        Display case legend if True
    
    save_figure : bool
        Save figure to file if True
    
    width : float
        Figure width in inches
    
    height : float
        Figure height in inches

    Returns
    -------
    None

    Notes
    -----
    Creates a single figure showing the specified quantity distribution
    along the airfoil surface for all cases (combinations of AoA and Re).
    
    Cases are plotted in different colors using the inferno colormap.
    Legend entries show AoA in degrees and Re number for each case.
    
    The plot uses standardized RCAIDE styling and axis formatting.

    See Also
    --------
    RCAIDE.Library.Plots.Common.set_axes : Standardized axis formatting
    RCAIDE.Library.Plots.Common.plot_style : RCAIDE plot styling
    RCAIDE.Library.Plots.Aerodynamics.plot_airfoil_boundary_layer_properties : Main plotting function
    """

    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    
    n_cpts   = len(ap.AoA[:,0])
    n_cases  = len(ap.AoA[0,:]) 
    
    fig   = plt.figure(qname.replace(" ", "_"))
    fig.set_size_inches(width,height) 
    axis  = fig.add_subplot(1,1,1)   
    
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,n_cases))      
    
    for i in range(n_cpts):   
        for j in range(n_cases): 
            case_label = 'AoA: ' + str(round(ap.AoA[i,j]/Units.degrees, 2)) + ', Re: ' + str(ap.Re[i,j]) 
            axis.plot( ap.x[i,j], q[i,j], color = line_colors[j], marker = ps.markers[0], linewidth = ps.line_width,  label =case_label)  
            axis.set_ylim([ylim_low,ylim_high]) 
     
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        
        # Adjusting the sub-plots for legend 
        fig.subplots_adjust(top=0.8)
        
    # set title of plot 
    title_text    = qname    
    fig.suptitle(title_text)
            
    if save_figure:
        plt.savefig(qname.replace(" ", "_") + file_type) 
          
    return  
  
   
           