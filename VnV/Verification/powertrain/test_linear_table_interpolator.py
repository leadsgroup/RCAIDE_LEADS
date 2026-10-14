# VnV/Verification/powertrain/test_linear_table_interpolator.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest
from scipy.interpolate import interp1d

from RCAIDE.Library.Attributes.Propellants.linear_table_interpolator import linear_table_interpolator, linear_table_set_interpolator

X = np.array([20.0, 25.0, 30.0, 40.0])
Y = np.array([[70.8, 0.10], [67.7, 0.12], [62.5, 0.15], [40.0, 0.25]])

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_matches_scipy_interp1d():
    x_new    = np.array([[20.0, 22.5], [33.3, 40.0]])
    expected = interp1d(X, Y[:, 0], kind='linear')(x_new)
    np.testing.assert_array_equal(linear_table_interpolator(X, Y[:, 0])(x_new), expected)
    for column, values in zip(Y.T, linear_table_set_interpolator(X, Y)(x_new)):
        np.testing.assert_array_equal(values, interp1d(X, column, kind='linear')(x_new))


@pytest.mark.parametrize("build", [lambda x: linear_table_interpolator(x, Y[:, 0]), lambda x: linear_table_set_interpolator(x, Y)])
def test_unsorted_table_raises(build):
    with pytest.raises(ValueError, match="strictly increasing"):
        build(X[::-1])


@pytest.mark.parametrize("interpolate", [linear_table_interpolator(X, Y[:, 0]), linear_table_set_interpolator(X, Y)])
@pytest.mark.parametrize("x_new, message", [(19.0, "below"), (41.0, "above")])
def test_out_of_range_raises(interpolate, x_new, message):
    with pytest.raises(ValueError, match=message):
        interpolate(np.array([30.0, x_new]))
