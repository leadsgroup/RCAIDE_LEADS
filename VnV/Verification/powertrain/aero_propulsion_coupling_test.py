# VnV/Verification/powertrain/aero_propulsion_coupling_test.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units

# package imports
import numpy as np
import time

# local imports
import sys
import os
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Electric_Twin_Otter import vehicle_setup, configs_setup

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------
def main():
    """ Aerodynamics-propulsion coupling (rotor slipstream on the wing, wing-induced velocities at the propeller
        discs) on a short cruise.
        Starboard and port propellers rotate in opposite directions, so the mirror-symmetric aircraft must
        give identical propeller performance on both sides.
    """
    ti = time.time()

    power_off = evaluate_cruise(coupled=False)
    power_on  = evaluate_cruise(coupled=True)
    print('Propeller power uncoupled [W]: ', power_off)
    print('Propeller power coupled   [W]: ', power_on)

    # mirror symmetry of the clockwise (starboard) and counter-clockwise (port) propellers
    diff_symmetry = np.abs(power_on[0] - power_on[1]) / power_on[0]
    print('Starboard/port power difference: ', diff_symmetry)
    assert diff_symmetry < 1e-6

    # the coupling changes the power required for the trimmed thrust
    print('Power change from coupling: ', power_on[0] / power_off[0] - 1)
    assert np.abs(power_on[0] - power_off[0]) / power_off[0] > 1e-4

    # regression
    power_true = 94985.59125124832
    diff_power = np.abs(power_on[0] - power_true) / power_true
    print('Power: ', power_on[0], ' difference: ', diff_power)
    assert diff_power < 1e-6

    elapsed_time = time.time() - ti
    print('Elapsed time (min): ', elapsed_time / 60)
    return

def evaluate_cruise(coupled):
    vehicle                                            = vehicle_setup('lithium_ion_nmc', None)
    vehicle.networks.electric.aero_propulsion_coupling = coupled

    propulsors                                              = vehicle.networks.electric.propulsors
    propulsors.starboard_propulsor.rotor.clockwise_rotation = True
    propulsors.port_propulsor.rotor.clockwise_rotation      = False
    for propulsor in propulsors:
        propulsor.rotor.use_2d_analysis           = True
        propulsor.rotor.number_azimuthal_stations = 8 # reducing the number of stations to speed up the test

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    mission  = mission_setup(analyses)
    results  = mission.evaluate()

    converters = results.segments.cruise.conditions.energy.converters
    power      = np.array([converters[propulsor.rotor.tag].power[0,0] for propulsor in propulsors])
    return power

def analyses_setup(configs):
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag,config in configs.items():
        analyses[tag] = base_analysis(config)
    return analyses

def base_analysis(vehicle):
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    weights = RCAIDE.Framework.Analyses.Weights.Electric_General_Aviation()
    analyses.append(weights)

    geometry                              = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.overwrite_reference = False
    analyses.append(geometry)

    aerodynamics                                       = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices  = 10 # reducing the number of vortices to speed up the test
    aerodynamics.settings.number_of_chordwise_vortices = 5  # reducing the number of vortices to speed up the test
    aerodynamics.settings.use_surrogate                = False
    analyses.append(aerodynamics)

    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)

    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)
    return analyses

def mission_setup(analyses):
    mission     = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
    Segments    = RCAIDE.Framework.Mission.Segments

    segment                                         = Segments.Cruise.Constant_Speed_Constant_Altitude()
    segment.tag                                     = 'cruise'
    segment.analyses.extend(analyses.base)
    segment.state.numerics.number_of_control_points = 3
    segment.altitude                                = 5000 * Units.ft
    segment.air_speed                               = 130 * Units.kts
    segment.distance                                = 10 * Units.nmi
    segment.initial_battery_state_of_charge         = 1.0

    segment.flight_dynamics.force_x = True
    segment.flight_dynamics.force_z = True

    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.pitch_angle.active           = True
    mission.append_segment(segment)
    return mission

if __name__ == '__main__':
    main()
