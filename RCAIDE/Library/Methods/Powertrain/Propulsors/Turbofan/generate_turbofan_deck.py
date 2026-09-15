# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/generate_turbofan_deck.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                                      import Data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    solve_turbofan_offdesign_robust, OffDesignMatchingError)
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan_offdesign_matching import (
    design_turbofan_offdesign_matching)

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  generate_turbofan_deck
# ----------------------------------------------------------------------------------------------------------------------
def generate_turbofan_deck(turbofan, altitude_range, mach_range, combustor_exit_temperature=None,
                                      isa_deviation=0.0, rating_code=0, fan_map=None,
                                      high_pressure_compressor_map=None, allow_unconverged_fallback=False):
    """
    Runs `solve_turbofan_offdesign_robust` across a grid of (altitude, Mach)
    flight conditions at a single throttle setting, and returns the result as
    a `Data` object of parallel arrays -- one entry per (altitude, Mach) grid
    point that converged. This is the offline "generate a deck algorithmically
    instead of importing one from GasTurb" path.

    This function has no pandas dependency and does not know about
    `Turbofan_Surrogate`'s specific column-name schema -- see Notes for how to
    hand its result to `Turbofan_Surrogate.build(dataframe=...)`, which does
    require a `pandas.DataFrame` with that schema.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
        An already-designed turbofan (`design_turbofan(turbofan)` already called).
    altitude_range : array_like
        Altitudes [m] to sweep.
    mach_range : array_like
        Mach numbers to sweep at each altitude.
    combustor_exit_temperature : float, optional
        Combustor exit (turbine inlet) stagnation temperature [K] for this
        throttle setting. Defaults to the turbofan's own design value (full
        throttle at whatever rating the design point represents).
    isa_deviation : float, optional
        ISA temperature deviation [K].
    rating_code : int, optional
        Rating code (Engine_Rating_Codes.png convention: 50 MTO, 45 MCO, 40
        MCL, 35 MCR, 20 FID, 0 not-a-rating/part-power), stamped onto every
        row generated. Does not change what this function computes -- it only
        labels the throttle setting swept, for `Turbofan_Surrogate`'s own
        rating-code bookkeeping downstream. A single call sweeps one throttle
        setting (one `combustor_exit_temperature`); build a full RC=0 part-
        power deck by calling this multiple times at different
        `combustor_exit_temperature` values and concatenating the results
        (e.g. `numpy.concatenate` field by field), matching the multi-row-
        per-flight-condition pattern `Turbofan_Surrogate`'s RC=0 throttle axis
        already expects.
    fan_map, high_pressure_compressor_map : RCAIDE.Library.Methods.Powertrain.Converters.Compressor.Generic_Compressor_Map, optional
        Passed through to `solve_turbofan_offdesign_robust`.
    allow_unconverged_fallback : bool, optional
        Passed through to `solve_turbofan_offdesign_robust`. Points that only
        converge via that last-resort tier are still included in the result
        (there is nowhere to flag "converged" in `Turbofan_Surrogate`'s
        schema), so leave this False (the default) unless the result's
        accuracy at those specific points has been checked and accepted --
        see that function's docstring for why it defaults to off.

    Returns
    -------
    Data
        altitude_m, mach_number, thrust_N, fuel_mass_flow_rate (kg/s),
        isa_deviation_k, rating_code -- each a 1-D numpy array, one entry per
        grid point that converged. Points that failed to converge (and
        weren't rescued by `allow_unconverged_fallback`) are omitted entirely,
        not written as NaN entries -- `Turbofan_Surrogate` interpolates over
        whatever rows exist, and a NaN row would just propagate NaN into any
        interpolation that touches it.

    Notes
    -----
    Feed this straight into `Turbofan_Surrogate.build()` via its `deck=`
    argument, which converts this result into the `pandas.DataFrame` schema
    `build()` requires internally (`ALT ft, XM, FN lbf, FF lb/h, ISA k, RC`):

        deck = generate_turbofan_deck(turbofan, altitude_range, mach_range)
        turbofan.surrogate = Turbofan_Surrogate().build(deck=deck)

    Pass `save_path=` too to also write the converted deck out as an Excel
    file (reusable later as a `deck_path=`):

        turbofan.surrogate = Turbofan_Surrogate().build(deck=deck, save_path="my_deck.xlsx")

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_Surrogate
    """
    design_constants, reference_point = design_turbofan_offdesign_matching(turbofan)
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
                result = solve_turbofan_offdesign_robust(
                    design_constants, reference_point, mach, static_temperature, static_pressure,
                    combustor_exit_temperature, fan_map=fan_map,
                    high_pressure_compressor_map=high_pressure_compressor_map,
                    allow_unconverged_fallback=allow_unconverged_fallback)
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
