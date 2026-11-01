# VnV/Verification/network_turbojet/test_turbojet_matching_floor.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                         import Units
from RCAIDE.Library.Methods.Powertrain                             import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet          import solve_turbojet_offdesign_robust
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Concorde import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_turbojet():
    vehicle = vehicle_setup()
    return list(vehicle.networks.fuel.propulsors)[0]


def throttle_sweep(turbojet, throttle, altitude, mach_number):
    """Thrust, fuel flow and below-floor flags, dry, over a throttle sweep."""
    network = RCAIDE.Framework.Networks.Fuel()
    bus     = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    network.distributors.append(bus)
    atmo    = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(altitude)
    state   = setup_operating_conditions(turbojet, bus, velocity_range=np.array([max(mach_number, 0.01) * atmo.speed_of_sound[0, 0]]), altitude=altitude)
    state.conditions.expand_rows(len(throttle))
    state.conditions.energy.propulsors[turbojet.tag].throttle[:, 0] = throttle
    state.unknowns.network['electrical_power'] = np.zeros((len(throttle), 1))
    turbojet.afterburner_active = False
    _, outputs, _, _ = turbojet.compute_performance(state, network)
    conditions = state.conditions.energy.propulsors[turbojet.tag]
    return outputs.thrust[:, 0], conditions.fuel_mass_flow_rate[:, 0], conditions.below_matching_floor[:, 0].astype(bool)

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_matching_burns_fuel_at_floor():
    turbojet = build_turbojet()
    matching = turbojet.offdesign_matching
    floor    = matching.matching_floor
    atmo     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    assert np.all(np.isfinite(floor.combustor_exit_temperature_fraction))
    for i in range(0, len(floor.altitude), 3):
        for j in range(0, len(floor.mach_number), 3):
            values = atmo.compute_values(floor.altitude[i])
            result = solve_turbojet_offdesign_robust(matching.design_constants, matching.reference_point, floor.mach_number[j],
                                                     values.temperature[0, 0], values.pressure[0, 0],
                                                     floor.combustor_exit_temperature_fraction[i, j] * matching.reference_point.Tt4)
            assert result.fuel_to_air_ratio > 0


def check_static_thrust_bounded_through_floor():
    turbojet = build_turbojet()
    throttle = np.linspace(0.0, 1.0, 101)
    thrust, fuel_flow, below_floor = throttle_sweep(turbojet, throttle, 0.0, 0.0)
    assert np.any(below_floor) and np.any(~below_floor)
    # near-static, thrust only goes below zero by the ram drag of the airflow at Mach 0.01
    assert np.all(np.isfinite(thrust)) and np.all(thrust >= -0.005 * turbojet.sealevel_static_thrust)
    assert np.all(np.diff(thrust) >= 0)
    # fuel flow is held at its floor value below the floor
    assert np.allclose(fuel_flow[below_floor], fuel_flow[below_floor][0])
    # slopes match across the floor; a fine step keeps the decay's curvature out of the finite difference
    floor = throttle[np.argmax(~below_floor)]
    fine  = np.linspace(floor - 0.01, floor + 0.01, 41)
    thrust, _, below_floor = throttle_sweep(turbojet, fine, 0.0, 0.0)
    i     = np.argmax(~below_floor)
    below = thrust[i - 1] - thrust[i - 2]
    above = thrust[i + 1] - thrust[i]
    assert np.isclose(below, above, rtol=0.15)


def check_supersonic_fuel_flow_never_negative():
    turbojet = build_turbojet()
    throttle = np.linspace(0.0, 1.0, 51)
    _, fuel_flow, _ = throttle_sweep(turbojet, throttle, 60000 * Units.ft, 2.02)
    assert np.all(fuel_flow >= 0)


def main():
    check_matching_burns_fuel_at_floor()
    check_static_thrust_bounded_through_floor()
    check_supersonic_fuel_flow_never_negative()
    return


if __name__ == '__main__':
    main()
