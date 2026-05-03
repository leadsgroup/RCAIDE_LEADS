
import numpy as np

def generate_3d_torus_points(origin, D, d, w, n_major, n_minor):
    """
    Generates 3D points for a torus (tire) geometry.

    The wheel axle is along Y; the wheel rolls in the XZ plane.
    Returns an array shaped (n_major+1, n_minor, 3) suitable for
    generate_vtk_object — the extra row closes the torus in the
    major-circle direction.

    Parameters
    ----------
    origin : array-like (3,)
        Centre of the torus [x, y, z].
    D : float
        Tire outer diameter; major radius = D/2.
    d : float
        Rim diameter; minor (tube cross-section) radius = d/2.
    w : float
        Tire width (informational; lateral extent is governed by d).
    n_major : int
        Azimuthal divisions around the wheel.
    n_minor : int
        Divisions around the tube cross-section.
    """
    r_inner = d / 2   # rim radius
    r_outer = D / 2   # tread radius
    h       = r_outer - r_inner   # radial height of cross-section

    # rectangular cross-section: 4 corners in (r, y_axial) space
    # traversed inner-near -> outer-near -> outer-far -> inner-far
    #   t = 0          h          h+w        2h+w     2h+2w (closed)
    perim    = np.array([0,       h,      h + w,   2*h + w,  2*(h + w)])
    corner_r = np.array([r_inner, r_outer, r_outer, r_inner,  r_inner ])
    corner_y = np.array([-w/2,   -w/2,    w/2,     w/2,     -w/2     ])

    t_vals  = np.linspace(0, 2*(h + w), n_minor, endpoint=False)
    cross_r = np.interp(t_vals, perim, corner_r)   # (n_minor,)
    cross_y = np.interp(t_vals, perim, corner_y)   # (n_minor,)

    # sweep cross-section around the wheel (axle along Y, rolls in XZ)
    theta = np.linspace(0, 2*np.pi, n_major + 1)[:, np.newaxis]  # (n_major+1, 1)
    r     = cross_r[np.newaxis, :]                                 # (1, n_minor)
    y_ax  = cross_y[np.newaxis, :]                                 # (1, n_minor)

    pts = np.zeros((n_major + 1, n_minor, 3))
    pts[:, :, 0] = r * np.cos(theta) + origin[0]
    pts[:, :, 1] = y_ax              + origin[1]
    pts[:, :, 2] = r * np.sin(theta) + origin[2]

    return pts
