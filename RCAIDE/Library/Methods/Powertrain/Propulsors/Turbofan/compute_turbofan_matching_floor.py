# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/compute_turbofan_matching_floor.py
#
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                                     import Data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import solve_turbofan_offdesign_robust
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.compute_matching_floor import compute_matching_floor

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbofan_matching_floor
# ----------------------------------------------------------------------------------------------------------------------
def compute_turbofan_matching_floor(design_constants, reference_point, altitudes, mach_numbers, margin=0.01, tolerance=1e-4):
    """
    Computes the matching floor of an off-design-matched turbofan: the lowest combustor exit temperature
    fraction the matching solver can represent at each altitude and Mach number, plus a margin.

    Parameters
    ----------
    design_constants : Data
        Turbofan design constants from design_turbofan_offdesign_matching.
    reference_point : Data
        Turbofan reference point from design_turbofan_offdesign_matching.
    altitudes : numpy.ndarray
        Floor altitudes [m].
    mach_numbers : numpy.ndarray
        Floor Mach numbers [-].
    margin : float, optional
        Combustor exit temperature fraction added above the floor.
    tolerance : float, optional
        Bisection tolerance on the combustor exit temperature fraction.

    Returns
    -------
    matching_floor : Data
        altitude, mach_number and combustor_exit_temperature_fraction (altitude x Mach), as consumed
        by compute_turbofan_performance_offdesign.

    Notes
    -----
    Below the floor the core or fan nozzle total pressure falls below ambient and the matching
    equations have no solution. The floor is a limit of the constant-efficiency matching model,
    not a certified idle setting: its thrust depends on the engine design point and can lie above
    or below the ICAO LTO idle of 7% rated thrust. The floor is evaluated at the design power
    offtake, the largest the engine is sized for. Points with no matched solution at any fraction
    take the highest floor found at that altitude.
    """
    return compute_matching_floor(solve_turbofan_offdesign_robust, design_constants, reference_point, altitudes, mach_numbers,
                                  margin=margin, tolerance=tolerance)
