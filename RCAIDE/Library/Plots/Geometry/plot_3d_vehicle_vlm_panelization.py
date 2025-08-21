# RCAIDE/Library/Plots/Geometry/plot_3d_vehicle_vlm_panelization.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Library.Plots.Geometry.Common.contour_surface_slice import contour_surface_slice  

import numpy as np  
import plotly.graph_objects as go 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  
def plot_3d_vehicle_vlm_panelization(vortex_distribution,
                                     alpha = 1.0,
                                     plot_axis = False,
                                     save_figure = False,
                                     show_wing_control_points = True,
                                     save_filename = "VLM_Panelization",
                                     axis_limit                  =  20,  
                                     show_figure = True):
    """
    Creates a 3D visualization of vehicle vortex lattice method (VLM) panelization.

    Parameters
    ----------
    vehicle : Vehicle
        RCAIDE vehicle data structure containing geometry information
        
    alpha : float, optional
        Transparency value between 0 and 1 (default: 1.0)
        
    plot_axis : bool, optional
        Flag to show coordinate axes (default: False)
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_wing_control_points : bool, optional
        Flag to display VLM control points (default: True)
        
    save_filename : str, optional
        Name of file for saved figure (default: "VLM_Panelization")
        
    axis_limit : float, optional
        Minimum x-axis plot limit (default: 20)  
        
    show_figure : bool, optional
        Flag to display the figure (default: True)

    Returns
    -------
    fig : plotly.graph_objects.Figure
        Figure handle containing the generated plot

    Notes
    -----
    Creates an interactive 3D visualization showing:
        - VLM panels on lifting surfaces
        - Control points (optional)
        - Customizable view and axis limits
    
    If vehicle's vortex distribution is not available, generates a new one with:
        - 25 spanwise vortices
        - 5 chordwise vortices
        - Linear spanwise spacing
        - No fuselage or nacelle modeling
    
    **Major Assumptions**
    
    * Lifting surfaces are represented by flat panels
    * Control points are at 3/4 chord of each panel
    * Vortex lines are at 1/4 chord of each panel
    
    **Definitions**
    
    'Control Point'
        Location where boundary condition is enforced
    'Vortex Line'
        Line of bound vorticity representing lift
    'Panel'
        Discrete element of lifting surface
    """

    # unpack vortex distribution 
    VD = vortex_distribution  

    camera        = dict(up=dict(x=0.5, y=0.5, z=1), center=dict(x=0, y=0, z=-.75), eye=dict(x=-1.5, y=-1.5, z=.8))
    plot_data     = []      

    # -------------------------------------------------------------------------
    # PLOT VORTEX LATTICE
    # -------------------------------------------------------------------------        
    n_cp      = len(VD.XA1[0])
    color_map = 'greys'
    for i in range(n_cp):  
        X = np.array([[VD.XA1[0][i],VD.XA2[0][i]],[VD.XB1[0][i],VD.XB2[0][i]]])
        Y = np.array([[VD.YA1[0][i],VD.YA2[0][i]],[VD.YB1[0][i],VD.YB2[0][i]]])
        Z = np.array([[VD.ZA1[0][i],VD.ZA2[0][i]],[VD.ZB1[0][i],VD.ZB2[0][i]]])           
        
        values      = np.ones_like(X) 
        verts       = contour_surface_slice(X,Y,Z,values,color_map,alpha)
        plot_data.append(verts)                 
  
  
    if  show_wing_control_points: 
        ctrl_pts = go.Scatter3d(x=VD.XC[0], y=VD.YC[0], z=VD.ZC[0],
                                    mode  = 'markers',
                                    marker= dict(size=6,color='red',opacity=0.8),
                                    line  = dict(color='red',width=2))
        plot_data.append(ctrl_pts)         
 
 
 
    fig = go.Figure(data=plot_data)
    fig.update_scenes(aspectmode   = 'auto',
                      xaxis_visible=plot_axis,
                      yaxis_visible=plot_axis,
                      zaxis_visible=plot_axis
                      )
    fig.update_layout( 
             width     = 1500,
             height    = 1500, 
             scene = dict(
                        xaxis = dict(backgroundcolor="grey", gridcolor="white", showbackground=plot_axis,
                                     zerolinecolor="white", range=[0,2 * axis_limit]),
                        yaxis = dict(backgroundcolor="grey", gridcolor="white", showbackground=plot_axis, 
                                     zerolinecolor="white", range=[-axis_limit,axis_limit]),
                        zaxis = dict(backgroundcolor="grey",gridcolor="white",showbackground=plot_axis,
                                     zerolinecolor="white", range=[-axis_limit,axis_limit])),             
             scene_camera=camera) 
    fig.update_coloraxes(showscale=False)
    fig.update_traces(opacity = alpha)
    if save_figure:
        fig.write_image(save_filename + ".png")
        
    if show_figure:
        fig.write_html( save_filename + '.html', auto_open=True)
    return fig
