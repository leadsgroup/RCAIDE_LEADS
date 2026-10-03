# RCAIDE/Library/Attributes/Propellants/linear_table_interpolator.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  linear_table_interpolator
# ----------------------------------------------------------------------------------------------------------------------
def linear_table_interpolator(x, y):
    """
    Linear interpolation of a property table that raises outside the table, matching
    scipy's interp1d(kind='linear', fill_value=None) bit for bit without its per-call
    overhead (cryogen property lookups run millions of times per mission).

    Parameters
    ----------
    x : ndarray
        Strictly increasing table abscissa.
    y : ndarray
        Table values at x.

    Returns
    -------
    interpolate : callable
        interpolate(x_new) -> ndarray of interpolated values, same shape as x_new.
    """
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    if not np.all(np.diff(x) > 0):
        raise ValueError("linear_table_interpolator: table abscissa must be strictly increasing.")
    x_min, x_max = x[0], x[-1]

    def interpolate(x_new):
        x_new = np.asarray(x_new, dtype=float)
        if np.any(x_new < x_min):
            raise ValueError(f"A value ({np.min(x_new)}) is below the interpolation range's minimum value ({x_min}).")
        if np.any(x_new > x_max):
            raise ValueError(f"A value ({np.max(x_new)}) is above the interpolation range's maximum value ({x_max}).")
        return np.asarray(np.interp(x_new, x, y))

    return interpolate



def linear_table_set_interpolator(x, Y):
    """
    Interpolates several property columns of one table at the same points with a single input
    conversion and bounds check. Each column still goes through np.interp, so results are identical
    to linear_table_interpolator on every platform (np.interp may use a fused multiply-add, which a
    vectorized numpy formula cannot reproduce). Raises outside the table.

    Parameters
    ----------
    x : ndarray
        Strictly increasing table abscissa, shape (n,).
    Y : ndarray
        Table values, shape (n, n_properties).

    Returns
    -------
    interpolate : callable
        interpolate(x_new) -> list of n_properties ndarrays, each shaped like x_new.
    """
    x = np.asarray(x, dtype=float)
    columns = [np.ascontiguousarray(c, dtype=float) for c in np.asarray(Y, dtype=float).T]
    if not np.all(np.diff(x) > 0):
        raise ValueError("linear_table_set_interpolator: table abscissa must be strictly increasing.")
    x_min, x_max = x[0], x[-1]

    def interpolate(x_new):
        x_new = np.asarray(x_new, dtype=float)
        if np.any(x_new < x_min):
            raise ValueError(f"A value ({np.min(x_new)}) is below the interpolation range's minimum value ({x_min}).")
        if np.any(x_new > x_max):
            raise ValueError(f"A value ({np.max(x_new)}) is above the interpolation range's maximum value ({x_max}).")
        return [np.interp(x_new, x, c) for c in columns]

    return interpolate
