# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/compute_matching_floor.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_matching_floor
# ----------------------------------------------------------------------------------------------------------------------
def compute_matching_floor(solve, design_constants, reference_point, altitudes, mach_numbers, margin=0.01, tolerance=1e-4):
    """
    Computes the matching floor of an off-design-matched gas turbine: the lowest combustor exit temperature
    fraction the matching solver can represent at each altitude and Mach number, plus a margin.

    Parameters
    ----------
    solve : callable
        Off-design solver called as solve(design_constants, reference_point, mach_number, static_temperature,
        static_pressure, combustor_exit_temperature) returning a result with fuel_to_air_ratio; raises
        OffDesignMatchingError where it has no solution.
    design_constants : Data
        Engine design constants passed to solve.
    reference_point : Data
        Engine reference point passed to solve; its Tt4 scales the fraction.
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
        altitude, mach_number and combustor_exit_temperature_fraction (altitude x Mach).

    Notes
    -----
    Below the floor a nozzle total pressure falls below ambient and the matching equations have no solution,
    or the requested combustor exit temperature is below the compressor exit temperature (negative fuel-air
    ratio), which happens in high-Mach flight where ram compression keeps the nozzle pressurized.
    The floor is a limit of the constant-efficiency matching model, not a certified idle setting. Points with
    no matched solution at any fraction take the highest floor found at that altitude.
    """
    # imported here: the Turbofan package imports Common
    from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import OffDesignMatchingError
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    upper      = 1.0
    lower      = 0.2
    def matched(fraction, mach_number, static_temperature, static_pressure):
        try:
            result = solve(design_constants, reference_point, mach_number, static_temperature, static_pressure,
                           fraction * reference_point.Tt4)
        except OffDesignMatchingError:
            return False
        # a burner cannot cool the flow: below the compressor exit temperature the fuel-air ratio turns negative
        return result.fuel_to_air_ratio > 0

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
            raise ValueError(f"No matched operating point at altitude {altitude:.0f} m for the matching floor.")
        fractions[i] = np.where(np.isnan(fractions[i]), np.nanmax(fractions[i]), fractions[i])

    matching_floor = Data(altitude                             = np.asarray(altitudes, dtype=float),
                          mach_number                          = np.asarray(mach_numbers, dtype=float),
                          combustor_exit_temperature_fraction  = fractions + margin)
    return matching_floor
