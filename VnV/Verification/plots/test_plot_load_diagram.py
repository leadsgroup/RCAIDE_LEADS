# VnV/Verification/plots/test_plot_load_diagram.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Library.Plots.Performance.plot_load_diagram import compute_loading_hull

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_loading_points_with_area_give_clipped_hull():
    CG   = np.array([10.0, 30.0, 30.0, 10.0])
    mass = np.array([100.0, 100.0, 300.0, 300.0])
    x, y = compute_loading_hull(CG, mass, 200.0)
    assert y.max() == 200.0 and x.min() == 10.0 and x.max() == 30.0


def test_collinear_loading_points_give_clipped_line():
    # one loading path (e.g. passengers only, no cargo bays): the hull is degenerate
    CG   = np.array([10.0, 15.0, 20.0, 25.0])
    mass = np.array([100.0, 150.0, 200.0, 250.0])
    x, y = compute_loading_hull(CG, mass, 200.0)
    np.testing.assert_allclose(x, [10.0, 15.0, 20.0])
    np.testing.assert_allclose(y, [100.0, 150.0, 200.0])


def test_single_loading_point_is_kept_below_max_mass():
    x, y = compute_loading_hull(np.array([12.0, 12.0]), np.array([150.0, 150.0]), 200.0)
    np.testing.assert_array_equal(x, [12.0, 12.0])
    np.testing.assert_array_equal(y, [150.0, 150.0])


def test_collinear_loading_points_above_max_mass_are_dropped():
    x, y = compute_loading_hull(np.array([10.0, 20.0]), np.array([300.0, 400.0]), 200.0)
    assert len(x) == 0 and len(y) == 0
