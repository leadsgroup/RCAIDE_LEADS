# VnV/Verification/powertrain/aero_propulsion_coupling_LL_test.py
#
# Created:  Oct 2026, H. Hussien

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
#  VALIDATION
# ----------------------------------------------------------------------------------------------------------------------
def main():
    """ Staged coupling check with Lifting_Line_Theory fidelity rotors on the real Electric_Twin_Otter:
        plain (no coupling) -> + wing-on-rotor coupling (rotor slipstream on the wing, wing-induced
        velocities at the propeller discs) -> + rotor-on-rotor coupling (mutual Biot-Savart induction
        between the starboard/port rotors' bound vortices and wakes). Each stage is checked for
        mirror symmetry between the clockwise (starboard) and counter-clockwise (port) propellers,
        and each new coupling term is checked to actually change the trimmed power by a nontrivial
        amount on top of the previous stage.
    """
    ti = time.time()

    power_plain = evaluate_cruise(wing_coupling=False, rotor_coupling=False)
    power_wing  = evaluate_cruise(wing_coupling=True,  rotor_coupling=False)
    power_both  = evaluate_cruise(wing_coupling=True,  rotor_coupling=True)
    print('Propeller power, no coupling     [W]: ', power_plain)
    print('Propeller power, +wing-on-rotor  [W]: ', power_wing)
    print('Propeller power, +rotor-on-rotor [W]: ', power_both)

    # mirror symmetry of the clockwise (starboard) and counter-clockwise (port) propellers, every stage
    for label, power in [('plain', power_plain), ('+wing', power_wing), ('+wing+rotor', power_both)]:
        diff_symmetry = np.abs(power[0] - power[1]) / power[0]
        print(f'Starboard/port power difference ({label}): ', diff_symmetry)
        assert diff_symmetry < 1e-6

    # wing-on-rotor coupling changes the power required for the trimmed thrust
    print('Power change from wing-on-rotor coupling: ', power_wing[0] / power_plain[0] - 1)
    assert np.abs(power_wing[0] - power_plain[0]) / power_plain[0] > 1e-4

    # rotor-on-rotor coupling changes the power required for the trimmed thrust, on top of wing coupling
    print('Power change from rotor-on-rotor coupling: ', power_both[0] / power_wing[0] - 1)
    assert np.abs(power_both[0] - power_wing[0]) / power_wing[0] > 1e-4

    elapsed_time = time.time() - ti
    print('Elapsed time (min): ', elapsed_time / 60)
    return

def evaluate_cruise(wing_coupling, rotor_coupling):
    vehicle                                           = vehicle_setup('lithium_ion_nmc', None)
    vehicle.networks.electric.wing_on_rotor_coupling  = wing_coupling
    vehicle.networks.electric.rotor_on_rotor_coupling = rotor_coupling

    propulsors                                              = vehicle.networks.electric.propulsors
    propulsors.starboard_propulsor.rotor.clockwise_rotation = True
    propulsors.port_propulsor.rotor.clockwise_rotation      = False
    for propulsor in propulsors:
        rotor                      = propulsor.rotor
        rotor.use_2d_analysis      = True
        rotor.fidelity             = 'Lifting_Line_Theory'
        rotor.rc                   = 0.05   # vortex core radius, fraction of R -- matches Tiltrotor_EVTOL.py
        rotor.wake_inputs          = None   # use lifting_line_performance()'s own validated defaults

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
    segment.state.numerics.mission_solver.type      = 'root_finder'  # much fewer evaluations than SLSQP -- LL is expensive per call
    segment.state.numerics.number_of_control_points = 2
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
