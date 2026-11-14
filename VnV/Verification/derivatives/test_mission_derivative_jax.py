# VnV/Verification/derivatives/test_mission_derivative_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # mission_derivative gives the derivative of a solved mission along a design change without solving it again: one
    # linear solve per segment with the compiled Jacobian, passed from segment to segment. Here the fuel burned by a
    # Cessna 172 over a climb, a cruise and a descent is differentiated with respect to a segment setting, a vehicle
    # value and a geometry value that passes through the aerodynamic surrogate, and each is compared with central
    # differences of missions solved again.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)

    vnv_folder   = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    start_folder = os.getcwd()
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    os.chdir(os.path.join(vnv_folder, "Verification", "network_internal_combustion_engine"))
    sys.path.insert(0, os.getcwd())

    mission = build_mission()
    mission.evaluate()

    # a segment setting and the takeoff mass reach the residual directly; the aspect ratio passes through the
    # aerodynamic surrogate, whose change is itself a finite difference of the setup
    check_fuel_derivative(mission, 'cruise_distance',   tolerance=1e-6)
    check_fuel_derivative(mission, 'takeoff_mass',      tolerance=1e-6)
    check_fuel_derivative(mission, 'wing_aspect_ratio', tolerance=1e-5)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_fuel_derivative(mission, parameter, tolerance, relative_step=1e-4):
    from RCAIDE.Library.Mission.Solver.mission_derivative import mission_derivative

    value = parameter_value(mission, parameter)
    step  = relative_step * value

    # without solving again: the mission built at the two moved designs, not solved
    derivative = mission_derivative(mission, build_mission(parameter, value + step), step, build_mission(parameter, value - step))
    mass       = 'state.conditions.weights.vehicle.mass'
    computed   = float(derivative[0].state[mass][0, 0] - derivative[-1].state[mass][-1, 0])

    # central differences of the mission solved at the two moved designs
    fuel = []
    for moved_value in (value + step, value - step):
        moved = build_mission(parameter, moved_value)
        moved.evaluate()
        fuel.append(fuel_burned(moved))
    expected = (fuel[0] - fuel[1]) / (2 * step)

    error = abs(computed - expected) / abs(expected)
    print(f"  d(fuel burned)/d({parameter}): mission derivative {computed:.8e}, solved again {expected:.8e}, relative difference {error:.1e}")
    assert error <= tolerance, f"{parameter}: mission derivative {computed} against {expected}"
    return


def fuel_burned(mission):
    segments = list(mission.segments.values())
    return float(segments[0].state.conditions.weights.vehicle.mass[0, 0] - segments[-1].state.conditions.weights.vehicle.mass[-1, 0])


def parameter_value(mission, parameter):
    vehicle = mission.segments.cruise.analyses.vehicle
    if parameter == 'cruise_distance':
        return float(mission.segments.cruise.distance)
    if parameter == 'takeoff_mass':
        return float(vehicle.mass_properties.takeoff)
    return float(vehicle.wings.main_wing.aspect_ratio)

# ----------------------------------------------------------------------------------------------------------------------
#  Mission
# ----------------------------------------------------------------------------------------------------------------------
def build_mission(parameter=None, value=None):
    """Climb, cruise and descent of the Cessna 172, with one design parameter set to value."""
    import ICE_test as test

    vehicle = test.vehicle_setup()
    # design values are floats: an integer would be part of the structure, not an input
    vehicle.mass_properties.takeoff = float(vehicle.mass_properties.takeoff)
    if parameter == 'takeoff_mass':
        vehicle.mass_properties.takeoff = value
    if parameter == 'wing_aspect_ratio':
        vehicle.wings.main_wing.aspect_ratio = value
    analyses = test.analyses_setup(test.configs_setup(vehicle))

    mission      = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag  = 'mission'
    Segments     = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()

    segment                 = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag             = "climb"
    segment.altitude_start  = 8000. * Units.feet
    segment.altitude_end    = 10000. * Units.feet
    segment.air_speed       = 90. * Units.knots
    segment.climb_rate      = 400. * Units['ft/min']
    add_segment(mission, segment, analyses)

    segment                 = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag             = "cruise"
    segment.altitude        = 10000. * Units.feet
    segment.air_speed       = 119. * Units.knots
    segment.distance        = 10. * Units.nautical_mile
    if parameter == 'cruise_distance':
        segment.distance    = value
    add_segment(mission, segment, analyses)

    segment                 = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag             = "descent"
    segment.altitude_end    = 8000. * Units.feet
    segment.air_speed       = 100. * Units.knots
    segment.descent_rate    = 400. * Units['ft/min']
    add_segment(mission, segment, analyses)
    return mission


def add_segment(mission, segment, analyses):
    """Force balance along and normal to the flight path, with the throttle and the pitch angle as controls."""
    segment.analyses.extend(analyses.base)
    segment.state.numerics.number_of_control_points                 = 8
    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['ice_propeller']]
    segment.assigned_control_variables.pitch_angle.active           = True
    mission.append_segment(segment)
    return


if __name__ == '__main__':
    main()
