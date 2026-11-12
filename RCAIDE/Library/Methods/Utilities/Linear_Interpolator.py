# RCAIDE/Library/Methods/Utilities/Linear_Interpolator.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.interpolate import RegularGridInterpolator, interp1d

from RCAIDE.Framework.Core import array_namespace, regular_grid_interpolation

# ----------------------------------------------------------------------------------------------------------------------
#  Linear_Interpolator
# ----------------------------------------------------------------------------------------------------------------------
class Linear_Interpolator():
    """
    Linear interpolation on a regular grid of one or two axes, extrapolating linearly beyond the ends.

    NumPy points are passed to scipy, unchanged from a plain RegularGridInterpolator or interp1d. JAX points use
    regular_grid_interpolation, the same lookup in jax.numpy, so the surrogate can be evaluated and differentiated
    inside jax.jit.

    Parameters
    ----------
    axes : list of 1D arrays
        Grid points along each axis, increasing.
    values : array
        Values on the grid, shape (len(axes[0]), ..., extra dimensions).
    """
    def __init__(self, axes, values):
        self.axes   = [np.asarray(axis, dtype=float) for axis in axes]
        self.values = np.asarray(values, dtype=float)
        if len(self.axes) == 1:
            self.scipy_interpolator = interp1d(self.axes[0], self.values, kind='linear', axis=0, bounds_error=False, fill_value='extrapolate')
        else:
            self.scipy_interpolator = RegularGridInterpolator(tuple(self.axes), self.values, method='linear', bounds_error=False, fill_value=None)

    def __call__(self, points):
        xp = array_namespace(points)
        if xp is np:
            return self.scipy_interpolator(points)
        if len(self.axes) == 1:
            # same output shape as interp1d: the points' shape followed by any extra value dimensions
            flat = regular_grid_interpolation(self.axes, self.values, xp.reshape(points, (-1, 1)))
            return xp.reshape(flat, points.shape + self.values.shape[1:])
        # same output shape as RegularGridInterpolator: one row per point, followed by any extra value dimensions
        return regular_grid_interpolation(self.axes, self.values, points)
