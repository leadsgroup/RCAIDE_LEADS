# RCAIDE/Library/Methods/Utilities/Linear_Interpolator.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import array_namespace, regular_grid_interpolation

# ----------------------------------------------------------------------------------------------------------------------
#  Linear_Interpolator
# ----------------------------------------------------------------------------------------------------------------------
class Linear_Interpolator():
    """
    Linear interpolation on a regular grid of one or two axes, extrapolating linearly beyond the ends, the same on
    NumPy and JAX (regular_grid_interpolation). It holds only its axes and values, so replacing them takes effect on
    the next call, and tables that are themselves inputs of a JAX evaluation (a design-dependent surrogate) are
    differentiated like any other input.

    Parameters
    ----------
    axes : list of 1D arrays
        Grid points along each axis, increasing.
    values : array
        Values on the grid, shape (len(axes[0]), ..., extra dimensions).
    """

    def __init__(self, axes, values):
        xp          = array_namespace(values, *axes)
        self.axes   = [xp.asarray(axis, dtype=float) for axis in axes]
        self.values = xp.asarray(values, dtype=float)

    def __call__(self, points):
        xp     = array_namespace(points, self.values, *self.axes)
        points = xp.asarray(points, dtype=float)
        if len(self.axes) == 1:
            # as interp1d: the points' shape followed by any extra value dimensions
            flat = regular_grid_interpolation(self.axes, self.values, xp.reshape(points, (-1, 1)))
            return xp.reshape(flat, points.shape + self.values.shape[1:])
        # as RegularGridInterpolator: points of shape (..., number of axes), followed by any extra value dimensions
        flat = regular_grid_interpolation(self.axes, self.values, xp.reshape(points, (-1, len(self.axes))))
        return xp.reshape(flat, points.shape[:-1] + self.values.shape[len(self.axes):])
