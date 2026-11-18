# RCAIDE/Library/Methods/Geometry/Solids/cut_section.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  cut_section
# ----------------------------------------------------------------------------------------------------------------------
def cut_section(x_upper, y_upper, x_lower, y_lower, start, end, number_of_stations=101):
    """
    The part of an airfoil between two chord fractions, as a closed polygon that stays inside the skin.

    Parameters
    ----------
    x_upper, y_upper, x_lower, y_lower : array
        Upper and lower surface of the airfoil per unit chord, each in either order along the chord.
    start, end : float
        Chord fractions of the cuts (front and rear spar).
    number_of_stations : int
        Stations between the cuts on each surface.

    Returns
    -------
    x, y : array
        The upper surface from start to end, then the lower surface from end to start. Every section cut with the
        same number of stations has the same vertices in the same order, so two sections can be lofted point to point.
    """
    xp       = array_namespace(start, end, y_upper, y_lower)
    upper    = np.argsort(np.asarray(x_upper))
    lower    = np.argsort(np.asarray(x_lower))
    stations = start + (end - start) * xp.linspace(0., 1., number_of_stations)
    y_top    = xp.interp(stations, xp.asarray(x_upper)[upper], xp.asarray(y_upper)[upper])
    y_bottom = xp.interp(stations, xp.asarray(x_lower)[lower], xp.asarray(y_lower)[lower])
    return xp.concatenate([stations, stations[::-1]]), xp.concatenate([y_top, y_bottom[::-1]])
