# RCAIDE/Library/Plots/Aerodynamics/plot_lift_distribution.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt 
import numpy as np 
# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_lift_distribution(results,
                           save_figure = False,
                           save_filename = "Lift_Distribution",
                           file_type = ".png",
                           width = 11, height = 7):
    """
    Generate plots of spanwise lift distribution for lifting surfaces.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
            - results.segments.conditions.aerodynamics.coefficients.lift with fields:
                - inviscid_wings_sectional : array
                    Sectional lift coefficients at control points

    save_figure : bool, optional
        Save figure to file if True, default False

    save_filename : str, optional
        Base name for saved figure files, default "Lift_Distribution"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 7

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing spanwise lift distribution

    Notes
    -----
    Creates figures showing:
        - Sectional lift coefficient (CLy) vs spanwise location
        - Separate plot for each timestep in each segment
        - Different wings distinguished by line colors:
            - Blue: Main wings
            - Red: Horizontal tails
            - Black: Other surfaces

    **Definitions**

    'Sectional Lift Coefficient'
        Non-dimensional lift force per unit span
    
    'Control Points'
        Points where circulation/lift is evaluated

    See Also
    --------
    RCAIDE.Library.Plots.Common.set_axes : Standardized axis formatting
    RCAIDE.Library.Plots.Common.plot_style : RCAIDE plot styling
    """   

    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                      'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    
    VD         = results.segments[0].analyses.aerodynamics.vehicle.vortex_distribution	 	
    n_w        = VD.n_w
    b_sw       = np.concatenate(([0],np.cumsum(VD.n_sw)))

    img_idx    = 1
    seg_idx    = 1
    for segment in results.segments.values():   	
        num_ctrl_pts = len(segment.conditions.frames.inertial.time)	
        for ti in range(num_ctrl_pts):  
            Clift_sectional = segment.conditions.aerodynamics.coefficients.lift.induced.spanwise[ti] 
            line = ['-b','-b','-r','-r','-k']
            fig  = plt.figure(save_filename + '_' + str(img_idx))
            fig.set_size_inches(8,8)  
            fig.set_size_inches(width,height)     
            axes = plt.subplot(1,1,1)
            for i in range(n_w): 
                y_pts = VD.Y_SW[b_sw[i]:b_sw[i+1]]
                z_pts = Clift_sectional[b_sw[i]:b_sw[i+1]]
                axes.plot(y_pts, z_pts, line[i] ) 
            axes.set_xlabel("Spanwise Location (m)")
            axes.set_title('$C_{Ly}$')  

            if save_figure: 
                plt.savefig( save_filename + '_' + str(img_idx) + file_type) 	
            img_idx += 1
        seg_idx +=1

    return fig 