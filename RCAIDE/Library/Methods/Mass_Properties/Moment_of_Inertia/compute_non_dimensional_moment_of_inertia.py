# RCAIDE/Library/Methods/Mass_Properties/Moment_of_Inertia/compute_non_dimensional_moment_of_inertia.py
#
# Created:  Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Non-Dimensional Moment of Inertia (I / mass)
# ----------------------------------------------------------------------------------------------------------------------
def compute_rounded_end_cylinder_non_dimensional_moi(radius, length):
    """Computes the non-dimensional moment of inertia tensor (I/mass)
    for a solid rounded-end cylinder (cylinder + hemispherical end caps).

    Parameters
    ----------
    radius : float
        Radius of the cylinder [m]
    length : float
        Length of the cylindrical section (excluding end caps) [m]

    Returns
    -------
    I_nd : numpy.ndarray
        3x3 non-dimensional moment of inertia tensor (I/mass).
        Cylinder axis is along x.
    """
    I_nd  = np.zeros((3, 3))
    V_cyl = np.pi * radius**2 * length
    V_sph = 4 / 3 * np.pi * radius**3
    V     = V_cyl + V_sph
    if V > 0:
        f_cyl = V_cyl / V
        f_sph = V_sph / V
        d_h   = length / 2 + (3 / 8) * radius
        I_nd[0][0] = 0.5 * f_cyl * radius**2 + 2 / 5 * f_sph * radius**2
        I_nd[1][1] = f_cyl * (radius**2 / 4 + length**2 / 12) + 2 / 5 * f_sph * radius**2 + f_sph * d_h**2
        I_nd[2][2] = I_nd[1][1]
    return I_nd


def compute_cuboid_non_dimensional_moi(length, width, height):
    """Computes the non-dimensional moment of inertia tensor (I/mass)
    for a solid cuboid.

    Parameters
    ----------
    length : float
        Length along x-axis [m]
    width : float
        Width along y-axis [m]
    height : float
        Height along z-axis [m]

    Returns
    -------
    I_nd : numpy.ndarray
        3x3 non-dimensional moment of inertia tensor (I/mass).
    """
    I_nd = np.zeros((3, 3))
    if length > 0 and width > 0 and height > 0:
        I_nd[0][0] = (width**2  + height**2) / 12
        I_nd[1][1] = (length**2 + height**2) / 12
        I_nd[2][2] = (length**2 + width**2)  / 12
    return I_nd
