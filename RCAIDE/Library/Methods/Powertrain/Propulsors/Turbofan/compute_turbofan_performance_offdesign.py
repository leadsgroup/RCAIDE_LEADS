# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/compute_turbofan_performance_offdesign.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core                                                      import Data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import (
    solve_turbofan_offdesign_robust, OffDesignMatchingError)

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbofan_performance_offdesign
# ----------------------------------------------------------------------------------------------------------------------
def compute_turbofan_performance_offdesign(turbofan, state, network=None, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes turbofan thrust and fuel flow by live off-design component
    matching (Mattingly Ch. 8 -- see `solve_turbofan_offdesign_robust`)
    instead of the fixed-pressure-ratio analytical cycle model or the table-
    driven surrogate. Dispatched from `compute_turbofan_performance` when
    `turbofan.offdesign_matching` is not None, checked before `turbofan.surrogate`.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
        Must have `turbofan.offdesign_matching` set to
        `Data(design_constants=..., reference_point=...)`, from
        `design_turbofan_offdesign_matching`.
    state : RCAIDE.Framework.Mission.Common.State
        Must have `conditions.freestream.mach_number/temperature/pressure`,
        and `conditions.energy.propulsors[turbofan.tag].throttle`.

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_propulsor_tag
        Same return signature as `compute_turbofan_performance`, for a common
        call site in `Turbofan.compute_performance`.

    Raises
    ------
    OffDesignMatchingError
        If the matching solver fails to converge at a control point AND
        `turbofan.offdesign_matching.idle_fallback` is not set. With no
        fallback configured, this fails loudly and stops the run rather than
        silently substituting an unflagged approximation for the failed point
        -- unchanged default behavior. If `idle_fallback` (a built
        `Turbofan_Surrogate`, queried with `rating_code='FID'`) is set, a
        point that fails to converge is routed to it instead of raising --
        see Notes. There is no solver-only fix for this: the matching
        equations are a closed-form power-balance model of a driven cycle,
        and deep part-power/idle is closer to windmilling (ram drag exceeds
        gross thrust -- confirmed against real Flight Idle deck data, not
        assumed), a different regime the equations don't describe at all.

    Notes
    -----
    Throttle is consumed differently here than by the analytical cycle model:
    the analytical model holds `combustor.turbine_inlet_temperature` fixed
    regardless of throttle and applies throttle as a post-hoc thrust
    multiplier downstream (`compute_thrust`). This solver instead needs an
    actual combustor exit temperature to match against, so throttle is used
    directly as a fraction of the reference point's own design Tt4:
    `Tt4 = reference_point.Tt4 * throttle`. This is a simplification (a real
    engine's throttle-to-Tt4 relationship is not perfectly linear), not a
    validated correlation -- consistent with how this solver was stress-
    tested throughout its development (as a Tt4-fraction sweep), but worth
    revisiting if a specific engine's real throttle schedule is known.

    Unlike the surrogate (which has no component-level station data to
    report and fills `noise_conditions` with NaN), this path computes real
    core/fan nozzle exit conditions as part of the matching solve -- except
    at a point routed to `idle_fallback`, which (being a surrogate query
    itself) has none either, so `noise_conditions` is NaN for exactly those
    points, real everywhere else.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.solve_turbofan_offdesign_robust
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan_offdesign_matching
    """
    conditions           = state.conditions
    turbofan_conditions  = conditions.energy.propulsors[turbofan.tag]
    noise_conditions      = conditions.aeroacoustics.propulsors[turbofan.tag]

    altitude            = conditions.freestream.altitude[:, 0]
    mach_number         = conditions.freestream.mach_number[:, 0]
    static_temperature  = conditions.freestream.temperature[:, 0]
    static_pressure     = conditions.freestream.pressure[:, 0]
    velocity            = conditions.freestream.velocity[:, 0]
    throttle            = turbofan_conditions.throttle[:, 0]

    design_constants = turbofan.offdesign_matching.design_constants
    reference_point  = turbofan.offdesign_matching.reference_point
    idle_fallback    = getattr(turbofan.offdesign_matching, 'idle_fallback', None)

    n = len(mach_number)
    thrust_N             = np.zeros(n)
    fuel_mass_flow_rate   = np.zeros(n)
    # NaN, not zero: a point routed to idle_fallback has no real station data (see Notes),
    # and NaN correctly signals that rather than a silently-wrong 0
    core_nozzle_exit_velocity            = np.full(n, np.nan)
    fan_nozzle_exit_velocity             = np.full(n, np.nan)
    core_nozzle_exit_static_temperature  = np.full(n, np.nan)
    fan_nozzle_exit_static_temperature   = np.full(n, np.nan)
    core_nozzle_exit_static_pressure     = np.full(n, np.nan)
    fan_nozzle_exit_static_pressure      = np.full(n, np.nan)
    core_nozzle_exit_stagnation_temperature = np.full(n, np.nan)
    fan_nozzle_exit_stagnation_temperature  = np.full(n, np.nan)
    core_nozzle_exit_stagnation_pressure    = np.full(n, np.nan)
    fan_nozzle_exit_stagnation_pressure     = np.full(n, np.nan)

    for i in range(n):
        combustor_exit_temperature = reference_point.Tt4 * throttle[i]
        try:
            result = solve_turbofan_offdesign_robust(
                design_constants, reference_point, mach_number[i], static_temperature[i], static_pressure[i],
                combustor_exit_temperature)
        except OffDesignMatchingError:
            if idle_fallback is None:
                raise
            # rating_code left at default (RC=0): idle_fallback's own build merges idle
            # shape into the SAME rating-code-0 throttle axis as its part-power sweep (see
            # aircraft_engine_mission_test.py's build_part_power_deck/setup_offdesign for
            # why), so the actual requested throttle carries through instead of collapsing
            # to a fixed idle value regardless of how much power was actually asked for
            F, FF = idle_fallback.query(np.array([altitude[i]]), np.array([mach_number[i]]),
                                         throttle=np.array([throttle[i]]))
            thrust_N[i]             = F[0]
            fuel_mass_flow_rate[i]  = FF[0]
            continue
        thrust_N[i]                                = result.thrust
        fuel_mass_flow_rate[i]                      = result.fuel_mass_flow_rate
        core_nozzle_exit_velocity[i]                = result.core_nozzle_exit_velocity
        fan_nozzle_exit_velocity[i]                 = result.fan_nozzle_exit_velocity
        core_nozzle_exit_static_temperature[i]      = result.core_nozzle_exit_static_temperature
        fan_nozzle_exit_static_temperature[i]       = result.fan_nozzle_exit_static_temperature
        core_nozzle_exit_static_pressure[i]         = result.core_nozzle_exit_static_pressure
        fan_nozzle_exit_static_pressure[i]          = result.fan_nozzle_exit_static_pressure
        core_nozzle_exit_stagnation_temperature[i]  = result.core_nozzle_exit_stagnation_temperature
        fan_nozzle_exit_stagnation_temperature[i]   = result.fan_nozzle_exit_stagnation_temperature
        core_nozzle_exit_stagnation_pressure[i]     = result.core_nozzle_exit_stagnation_pressure
        fan_nozzle_exit_stagnation_pressure[i]      = result.fan_nozzle_exit_stagnation_pressure

    thrust_vector      = np.zeros((n, 3))
    thrust_vector[:,0] = thrust_N

    TSFC           = np.zeros(n)
    positive       = thrust_N > 0
    gravity        = conditions.freestream.gravity[:, 0] if hasattr(conditions.freestream, 'gravity') \
                     else 9.80665 * np.ones(n)
    TSFC[positive] = fuel_mass_flow_rate[positive] * gravity[positive] / thrust_N[positive]

    power_propulsive = thrust_N * velocity

    # Compute forces and moments
    moment_vector      = 0*state.ones_row(3)
    moment_vector[:,0] = turbofan.origin[0][0] - center_of_gravity[0][0]
    moment_vector[:,1] = turbofan.origin[0][1] - center_of_gravity[0][1]
    moment_vector[:,2] = turbofan.origin[0][2] - center_of_gravity[0][2]
    moment              = np.cross(moment_vector, thrust_vector)

    # Pack turbofan outputs -- same fields as compute_turbofan_performance/_surrogate
    turbofan_conditions.thrust                            = thrust_vector
    turbofan_conditions.fuel_mass_flow_rate                = fuel_mass_flow_rate.reshape(-1,1)
    turbofan_conditions.thrust_specific_fuel_consumption   = TSFC.reshape(-1,1)
    turbofan_conditions.moment                             = moment
    turbofan_conditions.outputs.thrust                     = thrust_vector
    turbofan_conditions.outputs.moment                     = moment
    turbofan_conditions.outputs.power.propulsive           = power_propulsive.reshape(-1,1)

    if turbofan.combustor is not None and turbofan.combustor.fuel_data is not None:
        turbofan_conditions.inputs.power.chemical = \
            (fuel_mass_flow_rate * turbofan.combustor.fuel_data.lower_heating_value).reshape(-1,1)

    # noise_conditions schema match -- real values at points the live solver handled,
    # NaN at any point routed to idle_fallback (no station data there either) and for
    # fan angular velocity always (no equivalent computed by either path)
    noise_conditions.core_nozzle = Data(
        exit_static_temperature      = core_nozzle_exit_static_temperature.reshape(-1,1),
        exit_static_pressure         = core_nozzle_exit_static_pressure.reshape(-1,1),
        exit_stagnation_temperature  = core_nozzle_exit_stagnation_temperature.reshape(-1,1),
        exit_stagnation_pressure     = core_nozzle_exit_stagnation_pressure.reshape(-1,1),
        exit_velocity                = core_nozzle_exit_velocity.reshape(-1,1),
    )
    noise_conditions.fan_nozzle = Data(
        exit_static_temperature      = fan_nozzle_exit_static_temperature.reshape(-1,1),
        exit_static_pressure         = fan_nozzle_exit_static_pressure.reshape(-1,1),
        exit_stagnation_temperature  = fan_nozzle_exit_stagnation_temperature.reshape(-1,1),
        exit_stagnation_pressure     = fan_nozzle_exit_stagnation_pressure.reshape(-1,1),
        exit_velocity                = fan_nozzle_exit_velocity.reshape(-1,1),
    )
    noise_conditions.fan = Data(angular_velocity = np.full((n,1), np.nan))

    stored_results_flag   = True
    stored_propulsor_tag  = turbofan.tag

    return turbofan_conditions.inputs, turbofan_conditions.outputs, stored_results_flag, stored_propulsor_tag
