# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/compute_turbofan_performance_surrogate.py
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------------------------------------------------------
#  compute_turbofan_performance_surrogate
# ----------------------------------------------------------------------------------------------------------------------
def compute_turbofan_performance_surrogate(turbofan, state, network=None, center_of_gravity=[[0.0, 0.0, 0.0]]):
    """
    Computes turbofan thrust and fuel flow from a table-driven surrogate
    (turbofan.surrogate, a Turbofan_Surrogate instance) instead of the
    analytical cycle model. Dispatched from compute_turbofan_performance when
    turbofan.surrogate is not None.

    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan
        Must have turbofan.surrogate set to a built Turbofan_Surrogate.
    state : RCAIDE.Framework.Mission.Common.State
        Must have conditions.freestream.altitude/mach_number/velocity, and
        conditions.energy.propulsors[turbofan.tag].throttle/rating_code.

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_propulsor_tag
        Same return signature as compute_turbofan_performance, for a common
        call site in Turbofan.compute_performance.

    Notes
    -----
    noise_conditions is populated with the same schema compute_turbofan_performance
    builds (so readers like the aeroacoustics model don't hit an AttributeError), but
    every value is NaN -- a surrogate has no component-level station data to report.
    """
    conditions          = state.conditions
    turbofan_conditions = conditions.energy.propulsors[turbofan.tag]
    noise_conditions     = conditions.aeroacoustics.propulsors[turbofan.tag]

    altitude = conditions.freestream.altitude[:, 0]
    mach     = conditions.freestream.mach_number[:, 0]
    velocity = conditions.freestream.velocity[:, 0]
    gravity  = conditions.freestream.gravity[:, 0] if hasattr(conditions.freestream, 'gravity') \
               else 9.80665 * np.ones_like(mach)
    isa_dev  = conditions.freestream.delta_ISA[:, 0] if hasattr(conditions.freestream, 'delta_ISA') \
               else np.zeros_like(mach)
    throttle    = turbofan_conditions.throttle[:, 0]
    rating_code = turbofan_conditions.rating_code

    # target_SLS_thrust_N left at query()'s default -- design_thrust is cruise thrust, not SLS
    thrust_N, fuel_flow_kg_s = turbofan.surrogate.query(
        altitude_m=altitude, mach=mach, isa_dev=isa_dev, rating_code=rating_code, throttle=throttle)

    n = len(altitude)
    thrust_vector      = np.zeros((n, 3))
    thrust_vector[:,0] = thrust_N

    TSFC          = np.zeros(n)
    positive      = thrust_N > 0
    TSFC[positive] = fuel_flow_kg_s[positive] * gravity[positive] / thrust_N[positive]

    power_propulsive = thrust_N * velocity

    # Compute forces and moments
    moment_vector      = 0*state.ones_row(3)
    moment_vector[:,0] = turbofan.origin[0][0] - center_of_gravity[0][0]
    moment_vector[:,1] = turbofan.origin[0][1] - center_of_gravity[0][1]
    moment_vector[:,2] = turbofan.origin[0][2] - center_of_gravity[0][2]
    moment              = np.cross(moment_vector, thrust_vector)

    # Pack turbofan outputs
    turbofan_conditions.thrust                            = thrust_vector
    turbofan_conditions.fuel_mass_flow_rate                = fuel_flow_kg_s.reshape(-1,1)
    turbofan_conditions.thrust_specific_fuel_consumption   = TSFC.reshape(-1,1)
    turbofan_conditions.moment                             = moment
    turbofan_conditions.outputs.thrust                     = thrust_vector
    turbofan_conditions.outputs.moment                     = moment
    turbofan_conditions.outputs.power.propulsive           = power_propulsive.reshape(-1,1)

    if turbofan.combustor is not None and turbofan.combustor.fuel_data is not None:
        turbofan_conditions.inputs.power.chemical = (fuel_flow_kg_s * turbofan.combustor.fuel_data.lower_heating_value).reshape(-1,1)

    # noise_conditions schema match, values NaN -- see Notes above
    nan_column = np.full((n, 1), np.nan)
    nozzle_res = lambda: Data(
        exit_static_temperature      = nan_column,
        exit_static_pressure         = nan_column,
        exit_stagnation_temperature  = nan_column,
        exit_stagnation_pressure     = nan_column,
        exit_velocity                = nan_column,
    )
    noise_conditions.fan_nozzle  = nozzle_res()
    noise_conditions.core_nozzle = nozzle_res()
    noise_conditions.fan         = Data(angular_velocity = nan_column)

    stored_results_flag   = True
    stored_propulsor_tag  = turbofan.tag

    return turbofan_conditions.inputs, turbofan_conditions.outputs, stored_results_flag, stored_propulsor_tag
