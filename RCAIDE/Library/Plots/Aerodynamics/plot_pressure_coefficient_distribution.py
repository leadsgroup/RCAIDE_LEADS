# RCAIDE/Library/Plots/Performance/Aerodynamics/plot_surface_pressures.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units 
import matplotlib.pyplot as plt 
from matplotlib.colors import TwoSlopeNorm
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ---------------------------------------------------------------------------------------------------------------------- 
def plot_pressure_coefficient_distribution(results, 
                                            save_figure = False, 
                                            save_filename = "Pressure_Coefficient_Distribution",
                                            minimum_Cp = -1, 
                                            maximum_Cp =  1,
                                            discretization = 20, 
                                            file_type = ".png"):
    """
    Creates contour plots of differential surface pressure distributions (CP_lower -CP_upper) on aircraft lifting surfaces.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - segments[i].conditions.aerodynamics
                Aerodynamic data containing:
                    - coefficients.differential_surface_pressure[ti]
                        Pressure coefficient at each control point
            - segments[i].analyses.vehicle
                Vehicle data containing:
                    - vortex_distribution
                        Distribution data with:
                            - n_cw : array
                                Number of chordwise panels per wing
                            - n_sw : array
                                Number of spanwise panels per wing
                            - n_w : int
                                Number of wings
                            - XC, YC : arrays
                                Control point coordinates
                            - X : array
                                Surface point x-coordinates
                - wings : list
                    Wing components with:
                        - vertical : bool
                            Flag for vertical surfaces
                        - symmetric : bool
                            Flag for symmetric surfaces
            
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Surface_Pressure")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")

    Returns
    -------
    None

    Notes
    -----
    Creates visualization showing:
        - Surface pressure distributions
        - Spanwise pressure variations
        - Chordwise pressure variations
        - Wing geometry outlines
    
    **Definitions**
    
    'Pressure Coefficient'
        Non-dimensional pressure difference
    'Control Point'
        Location where pressure is evaluated
    'Lifting Surface'
        Wing, tail, or other aerodynamic surface
    'Planform'
        Top-view shape of lifting surface
    
    See Also
    --------
    RCAIDE.Library.Plots.Aerodynamics.plot_lift_distribution : Spanwise lift analysis
    RCAIDE.Library.Plots.Aerodynamics.plot_aerodynamic_coefficients : Overall coefficient plots
    """
     
    VD         = results.vortex_distribution
    n_cw       = VD.n_cw # number of panels chordwise (including control surfaces)
    n_sw       = VD.n_sw # number of panels spanwise  (including control surfaces)
    CP         = results.aerodynamics.coefficients.differential_surface_pressure
    b_pts      = np.concatenate(([0],np.cumsum(VD.n_sw[0]*VD.n_cw[0]))) 
    
    for ti in range(len(CP)): 
        figure_name = r'$\Delta$ $C_P$ Surface Distribution_' + str(ti+1)
        fig        = plt.figure(figure_name)
        axes       = plt.subplot(1, 1, 1)
        x_max      = max(VD.XC[ti]) + 2
        y_max      = max(VD.YC[ti]) + 2
        axes.set_ylim(x_max, 0)
        axes.set_xlim(-y_max, y_max)
        fig_w = 10.0
        fig_h = fig_w * (x_max / (2 * y_max)) if y_max > 0 else fig_w
        fig.set_size_inches(fig_w, fig_h)
           
        AoA = results.aerodynamics.angles.alpha[ti, 0]  
        CL  = results.aerodynamics.coefficients.lift.total[ti, 0]  
        CD  = results.aerodynamics.coefficients.drag.total[ti, 0]  
        figure_title =  'AoA: ' +  str(round(AoA/Units.degree, 4)) +  ', CL: ' +  str(round(CL, 2)) +  ', CD: ' +  str(round(CD, 4))
        fig.suptitle(figure_title, fontsize=16)
        for i in range(VD.n_w[0][0]):
            n_pts     = (n_sw[ti,i] + 1) * (n_cw[ti,i]+ 1)
            xc_pts    = VD.X[ti,i*(n_pts):(i+1)*(n_pts)]
            x_pts     = np.reshape(np.atleast_2d(VD.XC[ti,b_pts[i]:b_pts[i+1]]).T, (n_sw[ti,i],-1))
            y_pts     = np.reshape(np.atleast_2d(VD.YC[ti,b_pts[i]:b_pts[i+1]]).T, (n_sw[ti,i],-1))
            z_pts     = np.reshape(np.atleast_2d(CP[ti,b_pts[i]:b_pts[i+1]]).T, (n_sw[ti,i],-1))
            x_pts_p   = x_pts*((n_cw[ti,i]+1)/n_cw[ti, i]) - x_pts[0,0]*((n_cw[ti,i]+1)/n_cw[ti,i])  +  xc_pts[0]  
            norm      = TwoSlopeNorm(vmin=minimum_Cp, vcenter=0, vmax=maximum_Cp) 
            levels    = np.linspace(minimum_Cp, maximum_Cp, discretization)
            CS        = axes.contourf(y_pts,x_pts_p, z_pts,levels =levels, cmap = 'coolwarm', norm=norm,extend='both')
  
        # Set Color bar
        cbar = fig.colorbar(CS, ax=axes)
        cbar.ax.set_ylabel(r'$\Delta$ $C_{P}$', rotation =  0)
        plt.axis('off')
        plt.grid(None)

        if save_figure:
            plt.savefig( save_filename + '_' + str(ti+1) + file_type)  

    return
