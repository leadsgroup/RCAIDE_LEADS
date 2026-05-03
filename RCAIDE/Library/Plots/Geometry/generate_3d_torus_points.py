# RCAIDE/Library/Plots/Geometry/generate_3d_torus_points.py
#
# Created: Oct 2025, S Shekar 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
# RCAIDE imports 
import numpy as np 
# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  
import numpy as np

def generate_3d_torus_points(origin, D, d, w, tessellation = 24):
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
    curvature = 4

    # create cross section of rounded end torus with curvature control
    a = w/2
    b = h/2 
    theta    = np.linspace(0,2*np.pi,tessellation) 
    wheel_y_sec =  (abs((np.cos(theta)))**(2/curvature))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
    wheel_z_sec =  (abs((np.sin(theta)))**(2/curvature))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1) + r_inner + b
    wheel_x_sec = np.zeros_like(theta) 
     
    # revolve cross section around Y axis to create torus points
    phi = np.linspace(0, 2*np.pi, tessellation+1)
    pts = np.zeros((len(phi), len(theta), 3))
    for i in range(len(phi)):
        c, s = np.cos(phi[i]), np.sin(phi[i])
        pts[i,:,0] = origin[0] + wheel_x_sec * c - wheel_z_sec * s
        pts[i,:,1] = origin[1] + wheel_y_sec
        pts[i,:,2] = origin[2] + wheel_x_sec * s + wheel_z_sec * c   

    return pts
