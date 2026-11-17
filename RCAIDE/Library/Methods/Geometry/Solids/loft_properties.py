# RCAIDE/Library/Methods/Geometry/Solids/loft_properties.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import Data, array_namespace
from .polygon_properties import polygon_properties

# 3-point Gauss-Legendre nodes and weights on [0, 1]
GAUSS_NODES   = 0.5 + 0.5 * np.array([-np.sqrt(3. / 5.), 0., np.sqrt(3. / 5.)])
GAUSS_WEIGHTS = 0.5 * np.array([5. / 9., 8. / 9., 5. / 9.])

# ----------------------------------------------------------------------------------------------------------------------
#  loft_properties
# ----------------------------------------------------------------------------------------------------------------------
def loft_properties(inner_x, inner_z, outer_x, outer_z, inner_y, outer_y):
    """
    Volume, center of mass and inertia of the solid blended linearly between two parallel sections.

    Parameters
    ----------
    inner_x, inner_z : array
        Vertices of the inner section (x along the chord, z up), in order around it.
    outer_x, outer_z : array
        Vertices of the outer section, matched one to one with the inner ones.
    inner_y, outer_y : float
        Spanwise positions of the two sections.

    Returns
    -------
    properties : Data
        volume; center_of_gravity, array (3,) as (x, y, z); inertia_per_mass, array (3, 3), the inertia tensor about
        the center of gravity per unit mass (uniform density).

    Notes
    -----
    Each vertex moves on a straight line from the inner to the outer section, so the solid stays inside any wing
    whose skin is lofted the same way. Along the span the section area is quadratic and its second moments are
    quartic in the position, which 3-point Gauss-Legendre quadrature integrates exactly (exact to degree 5).

    References
    ----------
    M. Abramowitz and I. A. Stegun, Handbook of Mathematical Functions, 25.4.29 (Gauss-Legendre integration).
    """
    length = outer_y - inner_y
    volume = 0.
    first  = [0., 0., 0.]
    second = [0., 0., 0., 0., 0., 0.]     # xx, yy, zz, xy, yz, xz
    for t, w in zip(GAUSS_NODES, GAUSS_WEIGHTS):
        x = (1 - t) * inner_x + t * outer_x
        z = (1 - t) * inner_z + t * outer_z
        y = inner_y + t * length
        area, (sx, sz), (sxx, szz, sxz) = polygon_properties(x, z)
        dl        = w * length
        volume    = volume + dl * area
        first     = [first[0] + dl * sx, first[1] + dl * y * area, first[2] + dl * sz]
        second    = [second[0] + dl * sxx, second[1] + dl * y * y * area, second[2] + dl * szz,
                     second[3] + dl * y * sx, second[4] + dl * y * sz, second[5] + dl * sxz]
    return properties_from_moments(volume, first, second)


def properties_from_moments(volume, first, second):
    """Volume, center of gravity and inertia per unit mass from the volume integrals of 1, r and r r^T."""
    xp      = array_namespace(volume)
    center  = xp.stack([first[0], first[1], first[2]]) / volume
    xx, yy, zz, xy, yz, xz = [value / volume for value in second]
    moments = xp.stack([xp.stack([xx, xy, xz]), xp.stack([xy, yy, yz]), xp.stack([xz, yz, zz])]) - xp.outer(center, center)
    return Data(volume=volume, center_of_gravity=center, inertia_per_mass=xp.trace(moments) * xp.eye(3) - moments)
