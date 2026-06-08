# RCAIDE/Library/Plots/Geometry/plot_3d_vehicle_vlm_panelization.py
#
#
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pyvista as pv

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_3d_vehicle_vlm_panelization(vortex_distribution,
                                     alpha = 1.0,
                                     plot_axis = False,
                                     save_figure = False,
                                     show_wing_control_points = True,
                                     save_filename = "VLM_Panelization",
                                     axis_limit   =  100,
                                     panel_color  = 'grey',
                                     show_figure = True):
    """
    Creates a 3D visualization of vehicle vortex lattice method (VLM) panelization.

    Parameters
    ----------
    vortex_distribution : Data
        RCAIDE vortex distribution data structure containing panel corner coordinates

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
        Axis plot limit (default: 100)

    panel_color : str, optional
        Color of VLM panels (default: 'grey')

    show_figure : bool, optional
        Flag to display the figure (default: True)

    Returns
    -------
    plotter : pyvista.Plotter
        Plotter handle containing the generated plot

    Notes
    -----
    Creates an interactive 3D visualization showing:
        - VLM panels on lifting surfaces
        - Control points (optional)

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

    VD = vortex_distribution
    n_cp = len(VD.XA1[0])

    # -------------------------------------------------------------------------
    # Build quad mesh for VLM panels
    # -------------------------------------------------------------------------
    # Each panel has 4 corners ordered: A1, A2, B2, B1  (counter-clockwise quad)
    points = np.zeros((n_cp * 4, 3))
    faces  = np.zeros((n_cp, 5), dtype=int)  # [4, i0, i1, i2, i3]

    for i in range(n_cp):
        base = i * 4
        points[base + 0] = [VD.XA1[0][i], VD.YA1[0][i], VD.ZA1[0][i]]
        points[base + 1] = [VD.XA2[0][i], VD.YA2[0][i], VD.ZA2[0][i]]
        points[base + 2] = [VD.XB2[0][i], VD.YB2[0][i], VD.ZB2[0][i]]
        points[base + 3] = [VD.XB1[0][i], VD.YB1[0][i], VD.ZB1[0][i]]
        faces[i] = [4, base, base + 1, base + 2, base + 3]

    panel_mesh = pv.PolyData(points, faces.ravel())

    # -------------------------------------------------------------------------
    # Initialize plotter
    # -------------------------------------------------------------------------
    if save_figure:
        plotter = pv.Plotter(off_screen=True)
    else:
        plotter = pv.Plotter()

    plotter.add_mesh(panel_mesh, color=panel_color, opacity=alpha,
                     show_edges=True, edge_color='black', line_width=0.5)

    # -------------------------------------------------------------------------
    # Control points
    # -------------------------------------------------------------------------
    if show_wing_control_points:
        ctrl_pts = np.column_stack([VD.XC[0], VD.YC[0], VD.ZC[0]])
        ctrl_cloud = pv.PolyData(ctrl_pts)
        plotter.add_mesh(ctrl_cloud, color='red', point_size=6,
                         render_points_as_spheres=True, opacity=0.8)

    # -------------------------------------------------------------------------
    # Camera and display settings
    # -------------------------------------------------------------------------
    plotter.camera_position = [(-1.5 * axis_limit, -1.5 * axis_limit, 0.8 * axis_limit),
                                (axis_limit, 0, 0),
                                (0, 0, 1)]
    plotter.window_size = [1500, 1500]
    plotter.set_background('white')

    if not plot_axis:
        plotter.hide_axes()

    if save_figure:
        plotter.screenshot(save_filename + ".png")
    elif show_figure:
        plotter.show()

    return plotter
