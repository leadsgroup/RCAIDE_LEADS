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
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import solve_turbofan_offdesign_robust, OffDesignMatchingError

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
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    upper      = 1.0
    lower      = 0.2

    def matched(fraction, mach_number, static_temperature, static_pressure):
        try:
            solve_turbofan_offdesign_robust(design_constants, reference_point, mach_number, static_temperature,
                                            static_pressure, fraction * reference_point.Tt4)
            return True
        except OffDesignMatchingError:
            return False

    fractions = np.full((len(altitudes), len(mach_numbers)), np.nan)
    for i, altitude in enumerate(altitudes):
        atmo = atmosphere.compute_values(altitude)
        T0   = atmo.temperature[0, 0]
        P0   = atmo.pressure[0, 0]
        for j, mach_number in enumerate(mach_numbers):
            if not matched(upper, mach_number, T0, P0):
                continue
            if matched(lower, mach_number, T0, P0):
                fractions[i, j] = lower
                continue
            low, high = lower, upper
            while high - low > tolerance:
                middle = 0.5 * (low + high)
                if matched(middle, mach_number, T0, P0):
                    high = middle
                else:
                    low = middle
            fractions[i, j] = high

        if np.all(np.isnan(fractions[i])):
            raise ValueError(f"No matched turbofan operating point at altitude {altitude:.0f} m for the matching floor.")
        fractions[i] = np.where(np.isnan(fractions[i]), np.nanmax(fractions[i]), fractions[i])

    matching_floor = Data(altitude                             = np.asarray(altitudes, dtype=float),
                          mach_number                          = np.asarray(mach_numbers, dtype=float),
                          combustor_exit_temperature_fraction  = fractions + margin)
    return matching_floor
