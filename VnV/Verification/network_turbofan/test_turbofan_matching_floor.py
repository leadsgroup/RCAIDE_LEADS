# VnV/Verification/network_turbofan/test_turbofan_matching_floor.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                         import Units
from RCAIDE.Library.Methods.Powertrain                             import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan          import solve_turbofan_offdesign_robust
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_turbofan():
    vehicle = vehicle_setup()
    return list(vehicle.networks.fuel.propulsors)[0]


def throttle_sweep(turbofan, throttle, altitude, mach_number):
    """Thrust, fuel flow and below-floor flags at zero power offtake over a throttle sweep."""
    network = RCAIDE.Framework.Networks.Fuel()
    bus     = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    network.distributors.append(bus)
    atmo    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(altitude)
    state   = setup_operating_conditions(turbofan, bus, velocity_range=np.array([mach_number * atmo.speed_of_sound[0, 0]]), altitude=altitude)
    state.conditions.expand_rows(len(throttle))
    state.conditions.energy.propulsors[turbofan.tag].throttle[:, 0] = throttle
    state.unknowns.network['electrical_power'] = np.zeros((len(throttle), 1))
    _, outputs, _, _ = turbofan.compute_performance(state, network)
    conditions = state.conditions.energy.propulsors[turbofan.tag]
    return outputs.thrust[:, 0], conditions.fuel_mass_flow_rate[:, 0], conditions.below_matching_floor[:, 0].astype(bool)

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_matching_succeeds_at_floor():
    turbofan = build_turbofan()
    matching = turbofan.offdesign_matching
    floor    = matching.matching_floor
    atmo     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    assert np.all(np.isfinite(floor.combustor_exit_temperature_fraction))
    for i in range(0, len(floor.altitude), 3):
        for j in range(0, len(floor.mach_number), 3):
            values = atmo.compute_values(floor.altitude[i])
            solve_turbofan_offdesign_robust(matching.design_constants, matching.reference_point, floor.mach_number[j],
                                            values.temperature[0, 0], values.pressure[0, 0],
                                            floor.combustor_exit_temperature_fraction[i, j] * matching.reference_point.Tt4)


def check_thrust_continuous_and_monotone_through_floor():
    turbofan = build_turbofan()
    throttle = np.linspace(0.0, 0.8, 81)
    thrust, fuel_flow, below_floor = throttle_sweep(turbofan, throttle, 2000 * Units.ft, 0.2)
    assert np.any(below_floor) and np.any(~below_floor)
    assert np.all(np.isfinite(thrust))
    assert np.all(np.diff(thrust) > 0)
    # slope just below the floor matches the slope just above to within the finite-difference step
    i     = np.argmax(~below_floor)
    below = thrust[i - 1] - thrust[i - 2]
    above = thrust[i + 1] - thrust[i]
    assert np.isclose(below, above, rtol=0.15)
    # fuel flow is held at its floor value below the floor
    assert np.allclose(fuel_flow[below_floor], fuel_flow[below_floor][0])


def main():
    check_matching_succeeds_at_floor()
    check_thrust_continuous_and_monotone_through_floor()
    return


if __name__ == '__main__':
    main()
