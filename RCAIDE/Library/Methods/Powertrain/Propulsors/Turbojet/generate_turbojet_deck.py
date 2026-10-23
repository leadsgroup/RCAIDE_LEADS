# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbojet/generate_turbojet_deck.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                                      import Data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.Turbojet_OffDesign_Matching import (
    solve_turbojet_offdesign_robust, OffDesignMatchingError)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet_offdesign_matching import (
    design_turbojet_offdesign_matching)

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  generate_turbojet_deck
# ----------------------------------------------------------------------------------------------------------------------
def generate_turbojet_deck(turbojet, altitude_range, mach_range, combustor_exit_temperature=None,
                                      isa_deviation=0.0, rating_code=0, allow_unconverged_fallback=False):
    """
    Runs `solve_turbojet_offdesign_robust` across a grid of (altitude, Mach)
    flight conditions at a single throttle setting, and returns the result as
    a `Data` object of parallel arrays -- one entry per (altitude, Mach) grid
    point that converged. Same structure and purpose as `Turbofan.
    generate_turbofan_deck` (see its docstring for the full
    rationale); this is the turbojet-solver equivalent, feeding the exact
    same `Turbofan_Surrogate` interpolator (which has no turbofan-specific
    coupling at all -- it only consumes a plain thrust/fuel-flow deck).

    Parameters
    ----------
    turbojet : RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet
        An already-designed turbojet (`design_turbojet(turbojet)` already called).
    altitude_range : array_like
        Altitudes [m] to sweep.
    mach_range : array_like
        Mach numbers to sweep at each altitude.
    combustor_exit_temperature : float, optional
        Combustor exit (turbine inlet) stagnation temperature [K] for this
        throttle setting. Defaults to the turbojet's own design value.
    isa_deviation : float, optional
        ISA temperature deviation [K].
    rating_code : int, optional
        Rating code (`Engine_Rating_Codes.png` convention), stamped onto
        every row generated -- see `generate_turbofan_deck`'s own
        docstring for the full convention and multi-throttle build pattern.
    allow_unconverged_fallback : bool, optional
        Passed through to `solve_turbojet_offdesign_robust`. See
        `generate_turbofan_deck`'s own docstring for why this
        defaults to off.

    Returns
    -------
    Data
        altitude_m, mach_number, thrust_N, fuel_mass_flow_rate (kg/s),
        isa_deviation_k, rating_code -- each a 1-D numpy array, one entry per
        grid point that converged. Points that failed to converge are
        omitted entirely, not written as NaN entries.

    Notes
    -----
    Feed this straight into `Turbofan_Surrogate.build()` via its `deck=`
    argument, exactly as `generate_turbofan_deck`'s own docstring
    shows:

        deck = generate_turbojet_deck(turbojet, altitude_range, mach_range)
        turbojet.surrogate = Turbofan_Surrogate().build(deck=deck)

    `Turbojet` does not currently dispatch on a `.surrogate` attribute the
    way `Turbofan` does -- this deck is meant for `turbojet.offdesign_
    matching.idle_fallback` (a built `Turbofan_Surrogate` instance), the
    same role `Turbofan`'s `idle_fallback` plays in `compute_turbofan_
    performance_offdesign.py`.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.solve_turbojet_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_Surrogate
    """
    design_constants, reference_point = design_turbojet_offdesign_matching(turbojet)
    if combustor_exit_temperature is None:
        combustor_exit_temperature = reference_point.Tt4

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()

    altitude_m, mach_number, thrust_N, fuel_mass_flow_rate = [], [], [], []
    for altitude in altitude_range:
        atmo_data = atmosphere.compute_values(altitude, isa_deviation)
        static_temperature = float(np.ravel(atmo_data.temperature)[0])
        static_pressure    = float(np.ravel(atmo_data.pressure)[0])

        for mach in mach_range:
            try:
                result = solve_turbojet_offdesign_robust(
                    design_constants, reference_point, mach, static_temperature, static_pressure,
                    combustor_exit_temperature, allow_unconverged_fallback=allow_unconverged_fallback)
            except OffDesignMatchingError:
                continue

            altitude_m.append(altitude)
            mach_number.append(mach)
            thrust_N.append(result.thrust)
            fuel_mass_flow_rate.append(result.fuel_mass_flow_rate)

    deck = Data()
    deck.altitude_m           = np.array(altitude_m)
    deck.mach_number          = np.array(mach_number)
    deck.thrust_N             = np.array(thrust_N)
    deck.fuel_mass_flow_rate  = np.array(fuel_mass_flow_rate)
    deck.isa_deviation_k      = np.full(len(altitude_m), isa_deviation)
    deck.rating_code          = np.full(len(altitude_m), rating_code)
    return deck
