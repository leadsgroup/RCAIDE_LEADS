# VnV/Verification/network_turbofan/test_turbofan_offtake.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                 import Units
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan  import design_turbofan, solve_turbofan_offdesign_robust
import numpy as np
import sys
import os

sys.path.append(os.path.join(os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_turbofan_with_generator(design_power_offtake):
    """E190 turbofan redesigned with an integrated drive generator taking design_power_offtake at the design point."""
    turbofan                              = list(vehicle_setup().networks.fuel.propulsors)[0]
    generator                             = RCAIDE.Library.Components.Powertrain.Converters.Generator()
    generator.efficiency                  = 0.95
    generator.voltage_type                = 'AC'
    generator.nominal_voltage             = 115.0
    turbofan.integrated_drive_generator   = generator
    turbofan.design_power_offtake         = design_power_offtake
    turbofan.offdesign_matching           = None
    design_turbofan(turbofan)
    return turbofan


def solve(turbofan, mach_number, altitude, combustor_exit_temperature, shaft_power_offtake=None):
    matching = turbofan.offdesign_matching
    atmo     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(altitude)
    return solve_turbofan_offdesign_robust(matching.design_constants, matching.reference_point, mach_number,
                                           atmo.temperature[0, 0], atmo.pressure[0, 0], combustor_exit_temperature,
                                           shaft_power_offtake=shaft_power_offtake)


def fuel_at_thrust(turbofan, thrust, mach_number, altitude, shaft_power_offtake):
    """Fuel flow at a fixed thrust, by bisection on the combustor exit temperature."""
    Tt4R   = turbofan.offdesign_matching.reference_point.Tt4
    lo, hi = 0.6 * Tt4R, 1.2 * Tt4R
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if solve(turbofan, mach_number, altitude, mid, shaft_power_offtake).thrust < thrust:
            lo = mid
        else:
            hi = mid
    return solve(turbofan, mach_number, altitude, mid, shaft_power_offtake).fuel_mass_flow_rate

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_explicit_design_offtake_reproduces_design_point():
    # an explicit offtake equal to the design offtake must match the default (design-offtake) solution
    generator_power = 100e3
    turbofan        = build_turbofan_with_generator(generator_power)
    reference       = turbofan.offdesign_matching.reference_point
    shaft_power     = generator_power / turbofan.integrated_drive_generator.efficiency
    default         = solve(turbofan, reference.M0, turbofan.design_altitude, reference.Tt4)
    explicit        = solve(turbofan, reference.M0, turbofan.design_altitude, reference.Tt4, shaft_power_offtake=shaft_power)
    # to the design-point mass flow convergence of design_turbofan (1e-4)
    assert np.isclose(explicit.thrust, default.thrust, rtol=1e-4)
    assert np.isclose(explicit.fuel_mass_flow_rate, default.fuel_mass_flow_rate, rtol=1e-4)


def check_offtake_fuel_cost_is_thermodynamically_plausible():
    # extra fuel power for extra shaft power at fixed thrust is 1 / (incremental thermal efficiency); a gas
    # generator core lies between roughly 30% and 60%
    turbofan  = build_turbofan_with_generator(100e3)
    reference = turbofan.offdesign_matching.reference_point
    altitude  = turbofan.design_altitude
    thrust    = 0.9 * solve(turbofan, reference.M0, altitude, reference.Tt4).thrust
    lhv       = turbofan.combustor.fuel_data.lower_heating_value
    base      = fuel_at_thrust(turbofan, thrust, reference.M0, altitude, 100e3)
    extra     = fuel_at_thrust(turbofan, thrust, reference.M0, altitude, 200e3)
    incremental_efficiency = 100e3 / ((extra - base) * lhv)
    assert 0.30 < incremental_efficiency < 0.60


def main():
    check_explicit_design_offtake_reproduces_design_point()
    check_offtake_fuel_cost_is_thermodynamically_plausible()
    return


if __name__ == '__main__':
    main()
