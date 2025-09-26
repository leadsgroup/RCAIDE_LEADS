# RCAIDE/Library/Plots/Performance/Aerodynamics/plot_surface_pressures.py
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
def plot_surface_pressures(results, 
                          save_figure = False, 
                          save_filename = "Surface_Pressure", 
                          file_type = ".png"):
    """
    Creates contour plots of surface pressure distributions on aircraft lifting surfaces.

    Parameters
    ----------
    results : Results
        RCAIDE results data structure containing:
            - segments[i].conditions.aerodynamics
                Aerodynamic data containing:
                    - coefficients.surface_pressure[ti]
                        Pressure coefficient at each control point
            - segments[i].analyses.aerodynamics.vehicle
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
    
    vehicle    = results.segments[0].analyses.aerodynamics.vehicle
    VD         = results.segments[0].analyses.aerodynamics.settings.vortex_distribution
    n_cw       = VD.n_cw 
    n_sw       = VD.n_sw 
    b_pts      = np.concatenate(([0],np.cumsum(VD.n_sw[0]*VD.n_cw[0])))

    # Create a boolean for not plotting vertical wings
    idx        = 0
    plot_flag  = np.ones(int(VD.n_w[0]))
    for wing in vehicle.wings:
        if wing.vertical:
            plot_flag[idx] = 0
            idx += 1
        else:
            idx += 1
        if wing.vertical and wing.xz_plane_symmetric:
            plot_flag[idx] = 0
            idx += 1
        else:
            idx += 1

    img_idx    = 1
    seg_idx    = 1
    for segment in results.segments.values():
        num_ctrl_pts = len(segment.conditions.frames.inertial.time)
        for ti in range(num_ctrl_pts):
            CP         = segment.conditions.aerodynamics.coefficients.surface_pressure[ti] 
            fig        = plt.figure()
            axes       = plt.subplot(1, 1, 1)
            x_max      = max(VD.XC[ti]) + 2
            y_max      = max(VD.YC[ti]) + 2
            axes.set_ylim(x_max, 0)
            axes.set_xlim(-y_max, y_max)
            fig.set_size_inches(8,8)
            for i in range(int(VD.n_w[0])):
                n_pts     = (n_sw[ti,i] + 1) * (n_cw[ti,i]+ 1)
                xc_pts    = VD.X[ti,i*(n_pts):(i+1)*(n_pts)]
                x_pts     = np.reshape(np.atleast_2d(VD.XC[ti,b_pts[i]:b_pts[i+1]]).T, (n_sw[ti,i],-1))
                y_pts     = np.reshape(np.atleast_2d(VD.YC[ti,b_pts[i]:b_pts[i+1]]).T, (n_sw[ti,i],-1))
                z_pts     = np.reshape(np.atleast_2d(CP[b_pts[i]:b_pts[i+1]]).T, (n_sw[ti,i],-1))
                x_pts_p   = x_pts*((n_cw[ti,i]+1)/n_cw[ti, i]) - x_pts[0,0]*((n_cw[ti,i]+1)/n_cw[ti,i])  +  xc_pts[0] 
                color_map = plt.cm.get_cmap('jet')
                rev_cm    = color_map.reversed() 
                CS        = axes.contourf(y_pts,x_pts_p, z_pts, cmap = rev_cm,extend='both')

            # Set Color bar
            cbar = fig.colorbar(CS, ax=axes)
            cbar.ax.set_ylabel('$C_{P}$', rotation =  0)
            plt.axis('off')
            plt.grid(None)

            if save_figure:
                plt.savefig( save_filename + '_' + str(img_idx) + file_type)
            img_idx += 1
        seg_idx +=1

    return
