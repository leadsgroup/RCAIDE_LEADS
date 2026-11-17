# RCAIDE/Library/Methods/Geometry/Solids/polygon_properties.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from math import comb

from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  polygon_moment
# ----------------------------------------------------------------------------------------------------------------------
def polygon_moment(x, y, p, q):
    """
    The moment of a polygon, the integral of x^p y^q over its area, from its vertices.

    Parameters
    ----------
    x, y : array
        Vertex coordinates in order around the polygon, the last vertex not repeated.
    p, q : int
        Powers of x and y.

    Returns
    -------
    moment : float
        Positive for counter-clockwise vertices, negative for clockwise ones.

    Notes
    -----
    Exact for any simple polygon. Works on NumPy and JAX arrays.

    References
    ----------
    C. Steger, On the calculation of arbitrary moments of polygons, Technical Report FGBV-96-05, Forschungsgruppe
    Bildverstehen, Informatik IX, Technische Universitaet Muenchen, October 1996; formula as implemented in
    J. Burkardt, polygon_integrals, moment().
    """
    xp     = array_namespace(x, y)
    x_last = xp.roll(x, 1)
    y_last = xp.roll(y, 1)
    total  = 0. * x
    for k in range(p + 1):
        for l in range(q + 1):
            total = total + comb(k + l, l) * comb(p + q - k - l, q - l) * x ** k * x_last ** (p - k) * y ** l * y_last ** (q - l)
    return xp.sum((x_last * y - x * y_last) * total) / ((p + q + 2) * (p + q + 1) * comb(p + q, p))


# ----------------------------------------------------------------------------------------------------------------------
#  polygon_properties
# ----------------------------------------------------------------------------------------------------------------------
def polygon_properties(x, y):
    """
    Area, first moments and second moments of a polygon, whichever way round its vertices go.

    Parameters
    ----------
    x, y : array
        Vertex coordinates in order around the polygon, the last vertex not repeated.

    Returns
    -------
    area, first, second : tuple
        area; (integral of x, integral of y); (integral of x^2, integral of y^2, integral of x y), all over the area.
    """
    xp   = array_namespace(x, y)
    area = polygon_moment(x, y, 0, 0)
    sign = xp.sign(area)
    return (sign * area,
            (sign * polygon_moment(x, y, 1, 0), sign * polygon_moment(x, y, 0, 1)),
            (sign * polygon_moment(x, y, 2, 0), sign * polygon_moment(x, y, 0, 2), sign * polygon_moment(x, y, 1, 1)))
